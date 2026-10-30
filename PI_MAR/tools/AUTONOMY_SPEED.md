# Tốc độ tự quét và MOVE MentorPi

Profile 0,8 m/s dành cho M1 Mecanum **đã tháo xe kéo** và đã commission. Dừng `robot.zsh run` hoặc `navigation_session.zsh` hoàn toàn trước khi đổi cấu hình. Trong VS Code Attach vào container MentorPi, mở `/home/ubuntu/mediassist-mqtt-2026-10-08`.

```sh
cd /home/ubuntu/mediassist-mqtt-2026-10-08
pi/.venv/bin/python tools/configure_motion_speed.py --trailer-removed --speed 0.80 --reverse 0.80 --angular 0.80
pi/.venv/bin/python tools/configure_motion_speed.py --trailer-removed --speed 0.80 --reverse 0.80 --angular 0.80 --apply
```

Lệnh đầu chỉ kiểm tra. Lệnh thứ hai tạo bản sao lưu config/hardware và đổi đồng thời:

- `mapping.autonomy.speed_mps`, `max_reverse_mps`, `angular_rps` cho tự quét.
- `max_speed_mps`, `motion.max_reverse_speed_mps`, `navigation.max_speed_mps` cho cổng motor và MOVE Nav2.
- `motion.lidar_braking` để giảm tốc theo khoảng hở LiDAR cho mọi nguồn lệnh, kể cả MOVE; scan cũ hoặc không hợp lệ làm cổng motor dừng.

BE phải dùng hợp đồng MOVE có trần 0,8 và workflow mới `navigation-move-workflow.json` với `max_speed_mps=0.8`. Build lại BE rồi bấm **Chuẩn bị workflow MOVE** để publish phiên bản mới. Workflow cũ không tự đổi tốc độ.

Muốn giảm, chạy lại script với số nhỏ hơn cho `--speed`, `--reverse`, `--angular`; `--reverse` không được lớn hơn `--speed`. Tốc độ là trần độ lớn vector tiến/ngang, robot có thể chạy chậm hơn do gia tốc, costmap và LiDAR. Phần tự quét vẫn cần Nav2 frontier, map/odom/TF và khoảng hở đủ cho footprint. Thử ngắn trên sàn ở vùng thoáng trước khi dùng trong phòng.

Không sửa trực tiếp file `site.motion.*.json` bất kỳ: đường dẫn hardware profile đang dùng nằm trong `pi/config/mqtt.worker.json` ở trường `hardware_profile`. Script không sửa IP, TLS, camera, map hoặc journal.
