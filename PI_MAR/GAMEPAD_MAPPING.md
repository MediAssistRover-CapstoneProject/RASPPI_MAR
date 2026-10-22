# Tay cầm SHANWAN khi quét map

Chế độ này dùng driver ROS `joy` đã nhận trên Pi: 6 axes, 13 buttons.
Cần trái: `axes[1]` tiến/lùi, `axes[0]` quay. Không dùng nút L1 và không có
đi ngang trong bản này. Cần thả giữa 0,3 giây để nhận quyền điều khiển.

`robot.zsh run` tự mở một `joy_node` tên `mediassist_gamepad_joy`, phát vào
`/mediassist/gamepad/input`. Worker nhận Joy bằng QoS volatile/best effort,
depth 1, kiểm tra timestamp và xử lý vận tốc qua mapping tick 20 Hz. Worker
vẫn là chủ lease; không mở `joystick_control.launch.py` hãng và không thêm
publisher trực tiếp vào topic bánh xe. Camera, TTS và upload không được thêm
vào điều kiện di chuyển.

## Phạm vi giảm điều kiện

Preset cài đặt chọn `motion.us100_required=false`: mất echo hoặc số đo gần
của US100 không chặn chạy. Số đo vẫn được báo đúng, không tạo echo giả.
Camera lỗi cũng không làm dừng stack. Kiểm tra LiDAR/odom còn mới, commissioning,
topology, lease, watchdog và bảo vệ LiDAR phía sau khi lùi vẫn hoạt động.
Không có chế độ bỏ toàn bộ kiểm tra bánh xe.

Robot chỉ nhận cần khi phiên quét đã vào `MAPPING`, worker đã handshake MQTT
và không thực hiện step khác. Trước phiên: `WAIT_MAPPING`. Mở job không tự làm
robot chạy. Trả cần về giữa sau khi mở phiên, hết lượt hoặc sau lỗi/dừng.

Preset: tiến tối đa 0,10 m/s, lùi 0,08 m/s, quay 0,35 rad/s, có ramp tăng tốc.
Giới hạn phần cứng hiện có có thể thấp hơn. Thả cần đưa setpoint về zero ngay;
thời gian dừng cơ học cần thử thực tế. Mỗi lượt không về giữa giới hạn 3 giây,
tối đa cấu hình 5 giây. Giới hạn này ngăn giữ mãi một lệnh nếu receiver USB
tiếp tục autorepeat giá trị cũ. Mất bản tin ROS quá 0,25 giây làm ngừng cấp
lệnh, rồi gate/watchdog áp dụng thời hạn riêng; không cam kết thời gian phanh
cơ học bằng 0,25 giây.

Tắt nguồn tay cầm không nhất thiết làm USB receiver biến mất khỏi Linux.
`joy_node` autorepeat có thể tạo timestamp mới cho trạng thái HID cũ; chưa có
bằng chứng receiver này luôn báo mất sóng. Cần thử tắt tay cầm khi bánh nâng
khỏi mặt đất; giữ giới hạn lượt chạy. Cơ chế timeout ROS không chứng minh
phát hiện được mọi trường hợp mất sóng RF.

## Chọn nguồn, sau khi cài code và dừng stack

Trong container MentorPi:

```sh
cd /home/ubuntu/mediassist-mqtt-2026-10-08
pi/.venv/bin/python tools/configure_gamepad.py \
  --config pi/config/mqtt.worker.json --source gamepad --device-id 0 --us100 optional
zsh tools/robot.zsh run
```

Chỉ cần cấu hình một lần; các lần sau dùng `zsh tools/robot.zsh run`.
Trước khi chạy, Ctrl+C terminal `joy_node` chẩn đoán cũ. Không mở driver thứ hai.

Trong chế độ gamepad, lệnh chạy qua MQTT bị từ chối với
`MAPPING_CONTROL_SOURCE_GAMEPAD`, còn STOP đúng phiên vẫn được chấp nhận.
STOP khóa cần cho tới khi về giữa, nên autorepeat không lập tức ghi đè STOP.
Không bấm các nút tiến/lùi/quay trên web khi dùng tay cầm. Việc tạo phiên,
quét, xem bản đồ/camera, lưu/upload/commit và đóng job vẫn dùng HTML/BE cũ.

## Kiểm tra

Terminal container thứ hai, giữ terminal run mở:

```sh
cd /home/ubuntu/mediassist-mqtt-2026-10-08
tail -n 30 pi/state/stack/gamepad.log
tail -n 40 pi/state/stack/worker.log
ros2 topic echo /mediassist/gamepad/input --once --full-length
ros2 topic echo /mediassist/gamepad/status --once --full-length
ros2 topic echo /mediassist/safety_state --once --full-length
```

Sau START và thả cần: `READY`; đẩy cần: `DRIVING`; hết 3 giây:
`RUN_LIMIT_RETURN_NEUTRAL`. Không giữ cần lệch trong lúc mở job/handshake.
`SENSORS_NOT_READY` trong gamepad/status lúc này cần kiểm tra scan (US100
optional không gây lý do này); `ODOM_NOT_READY` cần kiểm tra controller/odom.
`JOY_PUBLISHER_COUNT`: không có đúng một publisher tên quy định trên input.

Thử đầu với toàn bộ bánh được kê chắc khỏi mặt đất: cần tiến/lùi/quay đúng
chiều, về giữa dừng, STOP web khi đang giữ cần vẫn dừng, giữ cần quá 3 giây
dừng, tháo receiver dừng, cắm lại phải về giữa. Sau đó mới thử tốc độ nhỏ trên
sàn trống. Không dùng lệnh publish Twist trực tiếp để vượt gate.

Nếu chiều quay ngược, dừng stack rồi:

```sh
pi/.venv/bin/python tools/configure_gamepad.py --source gamepad --angular-sign -1
zsh tools/robot.zsh run
```

Nếu chiều tiến ngược, tương tự dùng `--linear-sign -1`. Khi muốn đưa về chiều
dương ban đầu dùng `--angular-sign 1` hoặc `--linear-sign 1`.

## Trở lại nút chạy trên web

Kết thúc phiên đang quét, dừng stack, rồi:

```sh
pi/.venv/bin/python tools/configure_gamepad.py --source mqtt
zsh tools/robot.zsh run
```

Chuyển nguồn không tự đổi US100 về required. Chọn thêm `--us100 required` nếu
muốn US100 bắt buộc trở lại. Công cụ giữ nguyên token, URL, camera, calibration,
map/journal và flags, có backup cấu hình khi thay đổi.

## Giới hạn kiểm thử

Các test offline dùng Joy giả và bản source đã xác nhận, không quay bánh hay
đo độ mượt phần cứng. Máy phát triển chưa đăng nhập SSH Pi được qua khóa;
người vận hành cần chạy các bước kiểm tra trên robot.

Tham khảo cơ chế autorepeat và xử lý joystick: [source ROS joy](https://github.com/ros-drivers/joystick_drivers/blob/ros2/joy/src/joy.cpp).
