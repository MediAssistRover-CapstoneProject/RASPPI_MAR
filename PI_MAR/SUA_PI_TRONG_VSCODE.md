# Tự kiểm tra và sửa code Pi trong VS Code

19 file bản vá đã được cài vào Pi. Bước tuning cũng đã chạy thành công với `MECANUM_RUNTIME_APPLIED` trước khi bạn tắt robot: tự quét 0,18 m/s, quay 0,6 rad/s, native camera 8080, thumbnail MQTT tắt, SLAM normalizer 360 bins bật. Các đoạn dưới là code cần đối chiếu trong bản đã cài; không thêm một bản thứ hai nếu file đã có đúng nội dung.

## Mở đúng folder khi bật robot lại

1. VS Code → `Ctrl+Shift+P` → **Remote-SSH: Connect to Host…** → `ssh pi@192.168.1.28` → chọn Linux. Nếu đổi mạng, thay bằng IP SSH mới.
2. Trong Terminal của HOST Pi: `sudo docker start MentorPi`.
3. `Ctrl+Shift+P` → **Dev Containers: Attach to Running Container…** → chọn `MentorPi`.
4. **File → Open Folder** → `/home/ubuntu/mediassist-mqtt-2026-10-08`.
5. `Ctrl+P` để mở file theo các đường dẫn đầy đủ dưới đây. `Ctrl+S` lưu. Không sửa các bản stage trong `/home/pi` vì runtime dùng code trong container.

Nếu terminal container đang là root, chuyển sang ubuntu trước:

```sh
sudo -u ubuntu -H zsh
```

Sau đó:

```sh
source /home/ubuntu/.zshrc
cd /home/ubuntu/mediassist-mqtt-2026-10-08
```

Dừng `robot.zsh run` bằng Ctrl+C và chờ STOP trước khi sửa cấu hình hoặc đổi code. Tuning đã áp xong nên không cần chạy lại helper mỗi lần bật Pi.

## A. Camera: một nguồn RGB và server video hãng

Mở:

```text
/home/ubuntu/mediassist-mqtt-2026-10-08/pi/config/mqtt.worker.json
```

Tìm `mapping` → `camera`. Đối chiếu/sửa các key bên trong object camera hiện có; giữ các key khác của file:

```json
"enabled": true,
"topic": "/ascamera/camera_publisher/rgb0/image",
"stream_enabled": true,
"stream_backend": "web_video_server",
"stream_bind": "0.0.0.0",
"stream_port": 8080,
"stream_rate_hz": 20,
"stream_max_width": 640,
"stream_max_height": 480,
"stream_jpeg_quality": 60,
"mqtt_preview_enabled": false
```

Đây là các thuộc tính trong object hiện có, không phải toàn bộ JSON để ghi đè file. Sửa dấu phẩy theo vị trí thực tế. `stream_rate_hz=20` là giá trị cấu hình; FPS thực vẫn phụ thuộc driver/camera. Native server không tự tăng FPS nguồn.

Mở:

```text
/home/ubuntu/mediassist-mqtt-2026-10-08/pi/mediassist/mapping.py
```

Trong `MappingManager.__init__`, nhánh tạo thumbnail phải có điều kiện dưới đây. Không tạo thêm `CameraPreview` ở nơi khác:

```python
        if cfg.get('camera', {}).get('enabled') and cfg['camera'].get(
            'mqtt_preview_enabled',
            cfg['camera'].get('stream_backend') != 'web_video_server'
        ):
            from .camera_preview import CameraPreview
            self.camera = CameraPreview(
                adapter, cfg['camera'],
                lambda: self.state.get('state') == 'MAPPING'
            )
```

`self.camera = None` đã có phía trên trong hàm. `vendor_camera.py` đã gọi executable `web_video_server` cài sẵn của hãng; không cần thay bằng server Python hoặc viết lại driver camera. Supervisor sẽ dừng cây bringup hãng đã xác minh trước khi khởi động phần cứng MAR, rồi mở một video server. Nó không tự chiếm cổng đang bị một chương trình khác dùng.

Đường xem video:

```text
http://192.168.1.28:8080/stream?topic=/ascamera/camera_publisher/rgb0/image&type=mjpeg&width=640&height=480&quality=60
```

Khi chạy stack, không mở thêm `ros2 run web_video_server ...` hay Python stream 8081. Khi đo độ mượt, chỉ mở một viewer. Một publisher camera có thể có nhiều subscriber phục vụ các chức năng khác nhau; đó không đồng nghĩa nhiều driver camera.

## B. Tốc độ tự quét và cấu hình Mecanum

Cũng trong `mqtt.worker.json`, tìm `mapping.autonomy`. Chỉnh riêng các key đã có:

```json
"enabled": true,
"vehicle": "rigid",
"exploration_mode": "frontier",
"frontier_controller": "nav2",
"speed_mps": 0.18,
"angular_rps": 0.6,
"lidar_stop_distance_m": 0.1
```

Giữ object `footprint` hiện có, các giá trị clearance và những key khác. Preset này dành cho robot đã tháo xe kéo. Không dùng kích thước này cho xe kéo có khớp. Giới hạn phần cứng và giảm tốc theo vật cản vẫn có hiệu lực.

Tìm `mapping.scan_normalization`, phải là:

```json
"scan_normalization": {
  "enabled": true,
  "bins": 360
}
```

Normalizer chỉ dùng cho SLAM, không đổi raw scan của safety/AMCL hoặc tốc độ quay LiDAR vật lý.

Các file vận tốc đã sửa đồng bộ, không chỉ một giá trị Y:

```text
/home/ubuntu/mediassist-mqtt-2026-10-08/pi/mediassist/navigation_setup.py
/home/ubuntu/mediassist-mqtt-2026-10-08/pi/mediassist/ros_adapter.py
/home/ubuntu/mediassist-mqtt-2026-10-08/pi/mediassist/mapping.py
/home/ubuntu/mediassist-mqtt-2026-10-08/pi/mediassist/mapping_control.py
```

Trong `RosAdapter.__init__`, callback phải giữ đủ ba trục:

```python
        def nav_velocity(msg):
            with self.lock:
                self.nav_velocity_sample = (
                    (float(msg.linear.x), float(msg.linear.y), float(msg.angular.z)),
                    time.monotonic()
                )
        self.node.create_subscription(
            Twist, '/mediassist/nav2_cmd_vel', nav_velocity, 1
        )
```

Các hàm `navigation_velocity`, `publish_navigation_velocity` và mapping tick trong bản vá đã xử lý tuple `(x, y, z)`. Không đổi lại về tuple hai phần tử. Nav2 YAML và velocity smoother cũng đã có Y, giới hạn tổng vector XY và gia tốc. Khi chỉ đi ngang, lease vẫn phải bật. Không bật reverse tự động chỉ bằng cách cho `min_vel_x` âm vì còn cần xử lý guard phía sau và hành lang lùi.

## C. AMCL: không để timeout khóa READY vĩnh viễn

Mở:

```text
/home/ubuntu/mediassist-mqtt-2026-10-08/pi/mediassist/localization.py
```

Hàm hiện có phải giữ điều kiện `not status['ready']` trước khi áp timeout:

```python
def bootstrap_verdict(status, initialized, ever_ready, elapsed, timeout, error):
    if not initialized:
        return {**status, 'ready': False, 'reason': error}
    if not status['ready'] and not ever_ready and elapsed > timeout:
        return {
            **status,
            'ready': False,
            'reason': 'BOOTSTRAP_TIMEOUT_NEEDS_INITIAL_POSE'
        }
    return status
```

Mở:

```text
/home/ubuntu/mediassist-mqtt-2026-10-08/pi/mediassist/localization_node.py
```

Trong `tick`, verdict phải lấy từ evidence thật:

```python
        sample = self.evidence(now)
        status = self.gate.update(now, sample)
        status = bootstrap_verdict(
            status, self.initialized, self.ever_ready,
            now - self.boot_at if self.boot_at is not None else 0.,
            self.cfg['bootstrap_timeout_sec'], self.error
        )
        if status['ready']:
            self.ever_ready = True
```

`bootstrap_verdict` đã được import đầu file. Không thêm khối timeout cũ ghi `ready=False` vô điều kiện sau đoạn này. Không đặt `ready=True` hoặc `fresh=True` bằng tay: robot chỉ được nhận MOVE khi scan, odom, TF, covariance, độ khớp map và particle cùng đạt yêu cầu.

## D. Vì sao lúc quét thấy robot nhưng map đã lưu lại không thấy?

- Đang quét: SLAM tạo hệ tọa độ map của phiên đang chạy và biến đổi `map → odom`; từ đó lấy pose robot.
- Dừng/lưu job: vị trí trực tiếp của phiên quét bị ẩn theo thiết kế vì phiên đã kết thúc.
- Nạp map đã lưu: cần map server + AMCL trong một phiên navigation mới để định vị robot trên đúng map đó. Ảnh map không chứa vị trí hiện tại của robot.
- Phiên cũ `d4f7087a…` đã đóng. BE lưu trạng thái cuối `BOOTSTRAP_TIMEOUT_NEEDS_INITIAL_POSE` với `map_pose.fresh=false`, nên không có pose trực tiếp hợp lệ để vẽ.

Khi dùng map đã lưu: đóng job quét, dừng stack, tạo phiên map tạm mới trên UI và chạy đúng lệnh `navigation_session.zsh --session-id ...` UI cấp. Chọn global localization, hoặc seed vị trí thật/hướng thật nếu phòng khó phân biệt. Chờ READY rồi mới gửi MOVE tới endpoint. Không chạy mapping và navigation stack đồng thời.

Đường đi dữ liệu vị trí là:

```text
Odometry + IMU → EKF /odom
LiDAR + /odom → SLAM khi quét, hoặc AMCL khi dùng map đã lưu
Pose hợp lệ trên đúng map/session → MQTT → BE → mũi tên trên HTML
```

Đã xác nhận trên Pi có `/imu`, `/odom`, `/odom_raw`, một publisher RGB và một driver `/scan_raw`. EKF hãng có cấu hình nhận odom và IMU. Chưa xác nhận RF2O đang phát dữ liệu chỉ từ việc thấy tên topic trong graph.

Ưu tiên kiểm tra các nguồn hiện có trước khi thêm phần cứng. Source odom hãng được gửi kèm có phần tích phân từ vận tốc lệnh, nên không coi `/odom_raw` mặc nhiên là số đo encoder thực. Nếu cần nhận vị trí ban đầu tốt hơn, có thể dùng camera hiện có đọc AprilTag tại mốc đã biết trên map, sau khi hiệu chuẩn camera và khai báo vị trí mốc. Phần AprilTag chưa được tích hợp trong bản này.

Tham khảo chính thức: [Nav2: mapping và localization](https://docs.nav2.org/rolling/configuration_and_development/first_time_robot_setup_guide/sensors/mapping_localization/), [AprilTag pose estimation](https://github.com/AprilRobotics/apriltag#pose-estimation).

## E. Lưu và kiểm tra sau khi sửa

Trong terminal container, tại folder project và khi stack dừng:

```sh
PYTHONPATH="$PWD/pi:$PWD/pi/tools:$PWD/tools${PYTHONPATH:+:$PYTHONPATH}" \
  pi/.venv/bin/python -m unittest discover -s pi/tools -p test_mecanum_runtime.py -v
```

Test dùng dữ liệu mô phỏng/mock, không khởi động động cơ. Sau khi đã kiểm tra code/cấu hình:

```sh
zsh tools/robot.zsh run
```

Terminal khác trong container, khi stack đã chạy:

```sh
ros2 topic info /ascamera/camera_publisher/rgb0/image --verbose
pgrep -af web_video_server
tail -n 30 /home/ubuntu/mediassist-mqtt-2026-10-08/pi/state/stack/worker.log
```

Kiểm tra một RGB publisher và một executable server video thực. Nếu lệnh pgrep hiện cả wrapper `ros2 run` và executable, đọc cmdline để phân biệt launcher với hai server thực. Bản MAR dùng exec trực tiếp nên không cần wrapper riêng.

Trong mode map đã lưu, kiểm tra thêm:

```sh
ros2 lifecycle get /amcl
ros2 topic echo /mediassist/localization/status --once --full-length
```

AMCL không chạy trong mode quét mới là bình thường. Muốn xác nhận mũi tên trên map đã lưu, phải thử lại một phiên navigation đang READY; chưa có kết quả chạy thử đó trong lần kiểm tra này.
