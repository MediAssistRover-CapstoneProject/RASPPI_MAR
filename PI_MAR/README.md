# Runtime robot MentorPi

Code Python/ROS 2 Humble chạy trong container `MentorPi`. Thư mục dự án đang
dùng trên Pi: `/home/ubuntu/mediassist-mqtt-2026-10-08`. Trong repo Git,
thư mục tương ứng là `PI_MAR`.

## Bản cập nhật 2026-10-22

- SLAM giữ đường dẫn thư viện ROS khi chạy trong virtualenv; tạo graph mới cho
  phiên quét mới, loại map cũ và gửi kích thước, độ phân giải, tọa độ robot.
- MentorPi M1 hỗ trợ tiến/lùi và quay với tăng tốc theo ramp. Lùi cần cấu hình
  M1 cho phép và dữ liệu LiDAR phía sau hợp lệ, còn mới.
- Nhịp điều khiển mapping được lập lịch 20 Hz; vẫn giữ kiểm tra thời hạn lệnh.
- Upload map có thể tự lấy lại device token khi cấu hình device secret riêng.
- US100 có chế độ `required`/`optional`, nhất quán giữa worker, safety gate,
  kiểm tra khởi động và bench.
- Có công cụ kiểm tra/sửa DNS container từ DNS của mạng Pi host.

Code localization đã có trong repo được giữ lại. Đợt này không bật AMCL,
không gán map hay thiết lập vị trí ban đầu cho robot.

Thử camera HP60C 20 FPS và đo độ trễ: xem [CAMERA_TUNING.md](CAMERA_TUNING.md).

Điều khiển bằng cần trái SHANWAN trong phiên quét, chọn nguồn độc quyền và
US100 tùy chọn: xem [GAMEPAD_MAPPING.md](GAMEPAD_MAPPING.md).

MS200 đổi số tia/góc giữa các scan và cập nhật BE HTTPS khi đổi IP: xem
[MS200_SCAN.md](MS200_SCAN.md).

## Cấu hình riêng của robot

Repo không chứa cấu hình MQTT đang dùng, mật khẩu, device secret/token,
chứng chỉ riêng, SQLite journal hoặc các map vận hành của Pi. Khi cập nhật,
giữ nguyên `pi/config/mqtt.worker.json`, profile mà nó trỏ tới, virtualenv,
`pi/state` và `/home/ubuntu/mar-secrets`. Không thay profile đang dùng bằng
`pi/config/site.example.json`: file ví dụ có các cờ commissioning tắt.

Package metadata được đồng bộ thành `0.13.0`, theo `step_contract.VERSION`
đã có sẵn trong repo. Con số này không xác nhận Pi đã cài bản cập nhật.

## Vào container và chạy

PowerShell trên laptop, thay `IP_PI_HIEN_TAI` bằng IP SSH của Pi:

```powershell
ssh pi@IP_PI_HIEN_TAI
```

Trên Pi host:

```sh
sudo docker start MentorPi
sudo docker exec -it -u ubuntu \
  -w /home/ubuntu/mediassist-mqtt-2026-10-08 MentorPi zsh
```

Trong container, sau khi source đã được cập nhật và cấu hình riêng còn đầy đủ:

```sh
cd /home/ubuntu/mediassist-mqtt-2026-10-08
zsh tools/robot.zsh run
```

Giữ terminal này mở. `STACK_READY` là kiểm tra ROS; kiểm tra thêm
`BE MQTT session ready` trong `pi/state/stack/worker.log` và trạng thái robot
trên BE trước khi tạo job. Một terminal container khác có thể đọc:

```sh
cd /home/ubuntu/mediassist-mqtt-2026-10-08
tail -n 60 pi/state/stack/worker.log
tail -n 40 pi/state/stack/slam-engine.log
tail -n 30 pi/state/stack/us100.log
ros2 topic echo /mediassist/safety_state --once --full-length
```

Dừng stack bằng `Ctrl+C` ở terminal chạy robot, đợi các dòng `STOP` hoàn tất.
Không chạy thêm worker hoặc `us100_host_test.py` đồng thời với stack.

## Chọn US100 bắt buộc hay tùy chọn

Mặc định `motion.us100_required=true`, kể cả profile cũ chưa có field này.
Muốn dùng tùy chọn, dừng stack trước rồi chạy trong container:

```sh
cd /home/ubuntu/mediassist-mqtt-2026-10-08
pi/.venv/bin/python tools/configure_us100.py \
  --config pi/config/mqtt.worker.json --mode optional
zsh tools/robot.zsh run
```

Quay lại bắt buộc, cũng dừng stack trước:

```sh
pi/.venv/bin/python tools/configure_us100.py \
  --config pi/config/mqtt.worker.json --mode required
```

Công cụ tạo profile mới, đổi `hardware_profile` trong cấu hình worker và
lưu bản sao cấu hình cũ. Chỉ thay yêu cầu US100; giữ tốc độ, GPIO, hiệu chuẩn,
cờ commissioning và đường dẫn dữ liệu. Gọi lại cùng chế độ không tạo bản mới.

Ở chế độ optional, mất echo và vật ở gần US100 không chặn chuyển động;
số đo vẫn được báo đúng trạng thái. Kiểm tra LiDAR còn mới, lease, thời hạn
lệnh, commissioning và vùng phía sau khi lùi vẫn hoạt động. Điều này không
bổ sung tránh va chạm LiDAR toàn hướng. Step cần đo khoảng cách US100 vẫn
cần số đo thật; không tạo giá trị giả để hoàn thành step.

## DNS khi đổi mạng

Nếu worker báo `gaierror`, chạy trên **Pi host**. Lấy công cụ từ container:

```sh
sudo docker cp \
  MentorPi:/home/ubuntu/mediassist-mqtt-2026-10-08/tools/repair_pi_container_dns.py \
  /tmp/mar-repair-dns.py
sudo python3 /tmp/mar-repair-dns.py --container MentorPi
```

Nếu host phân giải được nhưng container lỗi, sửa bằng:

```sh
sudo python3 /tmp/mar-repair-dns.py --container MentorPi --apply
```

Công cụ sao lưu `resolv.conf`, chỉ đổi DNS container, kiểm tra lại và phục hồi
nếu sửa thất bại. Worker sẽ tự thử nối lại. DNS mạng có thể đổi sau khi chuyển
Wi-Fi hoặc tạo lại container; không cố định IP của broker cloud.

DNS không sửa được URL HTTPS cũ: khi IP laptop BE thay đổi, URL TTS/upload
trên Pi phải trỏ tới địa chỉ Pi truy cập được, chứng chỉ server phải có tên/IP
đó trong SAN, và CA tin cậy trên Pi phải khớp CA ký chứng chỉ. Device token
là thông tin xác thực API, không phải CA. Giữ kiểm tra TLS hoạt động.

## Kiểm thử offline

Chạy từ thư mục `PI_MAR`, với Python đã cài các dependency trong
`pi/pyproject.toml`:

```sh
python -m unittest discover -s pi/tools -p 'test_*.py'
python tools/test_pi_dns.py
```

Test không gửi lệnh tới động cơ hay sửa DNS thật. Một số bài cần OpenCV/NumPy,
fixture xuất từ BE hoặc môi trường PYTHONPATH đầy đủ sẽ được skip nếu thiếu.
Kiểm thử offline
không thay cho test cảm biến, phanh/dừng và truyền map thực tế trên Pi.
