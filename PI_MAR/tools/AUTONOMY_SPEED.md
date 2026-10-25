# Chỉnh tốc độ tự quét MentorPi

Áp dụng cho thân Mecanum đã cấu hình, tháo xe kéo. Chỉ thay cấu hình lúc `robot.zsh run` đã dừng hoàn toàn.

Trong VS Code đã Attach vào container MentorPi, mở folder:

`/home/ubuntu/mediassist-mqtt-2026-10-08`

Lệnh đổi trần tốc độ tự quét, không phát lệnh di chuyển:

```sh
cd /home/ubuntu/mediassist-mqtt-2026-10-08
pi/.venv/bin/python tools/configure_autonomy_speed.py --trailer-removed --speed 0.50 --angular 0.60 --apply
```

Thay `0.50` bằng `0.30`, `0.25` hoặc `0.18` khi muốn giảm. Bỏ `--apply` để chỉ xem cấu hình dự kiến. Script này không cần ROS, không kiểm tra cổng camera và không bật thêm tiến trình camera.

Mở file `pi/config/mqtt.worker.json` bằng Ctrl+P. Các trường cần biết nằm trong `mapping.autonomy`:

```json
{
  "speed_mps": 0.5,
  "angular_rps": 0.6,
  "slow_clearance_m": 0.8,
  "scan_max_age_sec": 0.2,
  "lidar_stop_distance_m": 0.1
}
```

Đây là các trường của object có sẵn, không thay cả file bằng đoạn JSON trên. Dùng script để đổi tốc độ vì nó cập nhật đồng thời `max_speed_mps` trong hardware profile, tạo bản sao lưu và kiểm tra cấu hình. Đường dẫn hardware profile hiện hành nằm ở `hardware_profile` trong file worker; không sửa một file `site.motion.*.json` bất kỳ.

Tốc độ 0,5 m/s là trần vector tiến/ngang, không phải 0,5 cho từng trục cộng lại. Nav2 còn giảm tốc để tránh vật cản, đổi hướng và tiếp cận đích. Trên 0,25 m/s yêu cầu Nav2 frontier, thân cứng, giảm tốc từ khoảng hở thân 0,8 m và scan không cũ quá 0,25 giây; helper chọn 0,2 giây. Ngưỡng LiDAR 0,1 m vẫn là dừng khẩn, không phải khoảng cách phanh đã đo tại 0,5 m/s.

Tay cầm, nút web điều khiển tay, giới hạn lùi, footprint, camera và mạng được giữ nguyên. Điều hướng endpoint trên map đã lưu vẫn có trần Nav2 0,25 m/s. Không thay API hoặc MQTT trên BE.

Sau khi sửa:

```sh
zsh tools/robot.zsh run
```

Chờ `STACK_READY` và `BE MQTT session ready`, tạo job quét mới, rồi bấm tự quét trên web. Việc chạy helper không tạo job hoặc tự chạy robot. Thử trên sàn trống để kiểm tra phanh và độ ổn định bản đồ; kiểm thử phần mềm không chứng minh hiệu năng cơ khí ở tốc độ mới.

Các file thực thi: `tools/configure_autonomy_speed.py` (cấu hình), `pi/mediassist/autonomous_mapping.py` (giới hạn và cảm biến), `pi/mediassist/configuration.py` (hardware), `pi/mediassist/navigation_setup.py` và `tools/mapping_nav2_parameters.py` (Nav2), `pi/mediassist/mapping.py` (vận tốc và lease tới gate). `tools/run_navigation_session.py` giữ trần endpoint tách khỏi tự quét.
