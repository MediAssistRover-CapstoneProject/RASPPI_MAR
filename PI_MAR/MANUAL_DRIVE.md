# MAR Manual Drive — 23/10/2026

## Code đã sửa

Đối chiếu source Hiwonder `peripherals/peripherals/joystick_control.py`: Mecanum dùng ly cho linear.x, lx cho linear.y, rx cho angular.z. Cần trái tiến/lùi/ngang; cần phải xoay thân. Cần phải ra tốc độ xoay, không đặt góc đích 90/180 độ. Launch hãng có mặc định .5 m/s, 2 rad/s; chưa đo tốc độ cơ khí thực.

Bản MAR cũ dùng axes[1] tiến/lùi và axes[0] quay, tốc độ .10/.08 m/s, .35 rad/s; dừng sau 3 giây giữ cần. SSH không tự làm giảm tốc độ motor.

Preset mới chỉ dành cho **robot đã tháo xe kéo**:

- Cần trái axes[1] tiến/lùi, axes[0] đi ngang; cần phải axes[2] xoay. Deadzone .15, một joy_node 20 Hz.
- Manual .25 m/s tiến/lùi/ngang, .8 rad/s quay. Vector chéo cũng giới hạn .25 m/s. Ramp .8 m/s², 2.5 rad/s²; về giữa dừng ngay.
- Không ngắt sau 3 giây. Input phải mới <=.25s. Khởi động/mất input/STOP/WF handoff cần về giữa để kích hoạt lại.
- Joystick dùng được trước khi tạo job và trong lúc MQTT kết nối/backoff. Một vòng điều khiển local trong worker, không chờ BE gia hạn từng lệnh.
- Khi giữ cần, heartbeat BUSY; Pi cũng từ chối bước WF mới nếu cần chưa về giữa. SAVE/STOP và bước WF giữ quyền riêng. Nếu mất broker trong khi quét, phiên quét vẫn abort/recovery theo luồng cũ; manual dùng lại sau khi về giữa.
- Dùng raw velocity -> gate -> odom_publisher hãng -> firmware motor hiện có. Không bật joystick_control hãng song song. Cùng cách dùng hai cần của source hãng; chưa đảm bảo cùng latency/tốc độ cơ khí thực.
- US100 diagnostic-only. LiDAR/odom freshness, gate/watchdog, graph check và các cờ commissioning giữ nguyên; không tự nâng cờ kiểm thử.
- Cho phép strafe Mecanum. Manual không tự bẻ lái tránh vật; lùi vẫn có rear LiDAR guard. Tự quét có giảm tốc/né vật và dừng khi bị chặn.
- Tự quét: rigid + frontier, .18 m/s, .5 rad/s, tối đa 300s. Không tự chuyển động: vẫn tạo job và bấm Bắt đầu tự quét. Dùng bao thân bảo thủ ±.22m trước/sau, ±.18m trái/phải từ base_footprint, lớn hơn thân 212×171mm đã cung cấp; đây không phải footprint đo chính xác. Không dùng khi xe kéo/đồ gắn vượt bao này còn gắn.
- BE/Web/Pi cùng trần manual .30 m/s, 1 rad/s; Pi giữ cap theo profile thực. Nav2 MOVE và WF đã lưu không tự tăng tốc. Web vẫn có lease ngắn/STOP và mapping job. Khi chọn gamepad, web khóa nút chạy, giữ STOP và tự quét.
- BE nhận manual_control trong heartbeat và hiển thị nguồn/tốc độ trên HTML. Không thêm StepType, không đổi START -> WAIT -> SAVE -> upload -> COMMIT -> STOP. BE mới nhận Pi cũ; cập nhật BE trước Pi.
- Camera hãng 8080 có trong bộ vá; cấu hình backend là bước riêng. Chưa thêm chuyển tiếp video cloud.

## Cài BE trước, Pi sau

Trả cần về giữa, lưu/kết thúc job. Ctrl+C terminal robot, đợi toàn bộ STOP. Ctrl+C server video hãng mở tay nếu còn chạy.

### Windows PowerShell

Source BE cũ đã sửa tại đường dẫn sau. Không cần đổi .env/CA/token.

```powershell
Set-Location 'C:\Users\LENOVO\Downloads\MAR_EMQX_Demo\BE_MAR'
docker compose -p mar-emqx-demo up -d --build backend
```

Chờ BE khởi động rồi chạy:

```powershell
curl.exe --fail http://127.0.0.1:8080/actuator/health
docker compose -p mar-emqx-demo exec -T reverse-proxy nginx -s reload
scp 'C:\Users\LENOVO\Downloads\MAR_ManualDrive_PI_2026_10_23.zip' pi@192.168.1.22:/tmp/
ssh pi@192.168.1.22
```

### HOST Pi

```sh
sudo docker cp /tmp/MAR_ManualDrive_PI_2026_10_23.zip MentorPi:/tmp/MAR_ManualDrive_PI_2026_10_23.zip
sudo docker exec -it -u ubuntu -w /home/ubuntu/mediassist-mqtt-2026-10-08 MentorPi zsh
```

### Container MentorPi

```sh
cd /home/ubuntu/mediassist-mqtt-2026-10-08
PATCH_DIR=$(mktemp -d /tmp/mar-manual-XXXXXX)
pi/.venv/bin/python -m zipfile -e /tmp/MAR_ManualDrive_PI_2026_10_23.zip "$PATCH_DIR"
pi/.venv/bin/python "$PATCH_DIR/install_pi_patch.py"
pi/.venv/bin/python "$PATCH_DIR/install_pi_patch.py" --apply
pi/.venv/bin/python tools/configure_bare_mecanum.py --trailer-removed --apply
pi/.venv/bin/python tools/configure_camera_backend.py --backend web_video_server --apply
zsh tools/robot.zsh run
```

Installer chỉ ghi code, có hash baseline/backup/rollback. Helper tiếp theo mới tạo hardware profile mới và backup config. Giữ maps, secrets, journal. Nếu CODE_BASELINE_MISMATCH, gửi tên file, không bỏ kiểm tra. Cờ commissioning chưa pass vẫn giữ nguyên, không tự bật.

Nếu CAMERA_PORT_8080_BUSY, dừng terminal web_video_server mở tay bằng Ctrl+C rồi chạy lại helper; không kill toàn bộ ROS.

## Test thực

1. Tháo xe kéo, hai cần về giữa khi run. Chưa tạo job: thử tiến/lùi/ngang bằng cần trái, xoay bằng cần phải. Kiểm tra hướng trước khi gạt hết hành trình.
2. Giữ cần 8–10 giây ở khu trống: không còn ngắt 3 giây. Thả cần dừng. Rút USB phải dừng. Fresh ROS timestamp không chứng minh RF còn sống nếu dongle lặp HID cũ; phải test tắt tay cầm thực. Bản vá không phát hiện mọi dạng mất sóng RF này.
3. Về giữa, tạo mapping job trên HTML, dùng joystick đi quét. Nút web chạy bị khóa khi nguồn gamepad, STOP vẫn hoạt động.
4. Tự quét: để cần giữa -> Bắt đầu tự quét. Phải thấy available=true, vehicle=rigid, exploration_mode=frontier. Gạt cần lúc auto sẽ dừng auto; bấm STOP rồi trả cần giữa để về manual. Không tự nối tiếp auto sau override.
5. Lưu: đưa cần giữa -> Lưu bản đồ và kết thúc -> đợi SUCCEEDED. SAVE/STOP không được bị joystick phát lệnh chuyển động xen vào.

Camera: http://192.168.1.22:8080/stream?topic=/ascamera/camera_publisher/rgb0/image&type=mjpeg&width=640&height=480&quality=60

Terminal container khác để đọc:

```sh
cd /home/ubuntu/mediassist-mqtt-2026-10-08
tail -n 45 pi/state/stack/worker.log
ros2 topic echo /mediassist/gamepad/status --once --full-length
ros2 topic echo /mediassist/safety_state --once --full-length
```

Status cần có local_manual=true; requested/applied_velocity có x,y,yaw. Nếu dừng, đọc reason cụ thể.

## Tốc độ / nguồn điều khiển

Stack phải dừng trước khi cấu hình. Mức trần thử thêm .30 m/s, 1 rad/s:

```sh
pi/.venv/bin/python tools/configure_bare_mecanum.py --trailer-removed --linear 0.30 --angular 1.0 --auto-speed 0.18 --apply
```

Dùng các nút web (vẫn trong mapping job):

```sh
pi/.venv/bin/python tools/configure_bare_mecanum.py --trailer-removed --source mqtt --apply
```

Trở về gamepad:

```sh
pi/.venv/bin/python tools/configure_bare_mecanum.py --trailer-removed --source gamepad --apply
```

Helper không đổi IP/chứng chỉ, camera backend, servo, endpoint hoặc nội dung map. Không dùng preset khi còn xe kéo.

## Trạng thái giao bản

Code/bản vá trên laptop; chưa cài hoặc thử chuyển động vật lý trên Pi, chưa restart BE đang chạy. Kiểm thử ghi ở verification.json. Chưa commit/push.
