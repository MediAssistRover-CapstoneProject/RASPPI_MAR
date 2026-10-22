# Thử camera HP60C ở 20 FPS và đo độ trễ

## Mục tiêu và giới hạn

Camera trong log là `AS_CAM_HP60C` (Angstrong Nuwa-HP60C). Hiwonder công bố
RGB tối đa 1920×1080 @ 20 FPS, depth 640×480 @ 20 FPS; RGB 640×480 @ 20 FPS
là một cấu hình được hãng khuyến nghị:
https://www.hiwonder.com/products/ros-real-sense-depth-camera

File vendor đã đối chiếu từ bản source lưu trên laptop:
`src/peripherals/launch/include/ascamera.launch.py`. Node `ascamera_node`
dùng parameter **`fps`**, đang đặt `15`. Log in ra `set_fps` nhưng tên
parameter trong launch là `fps`.

Thử nghiệm mới:

| Lớp | Cấu hình thử |
| --- | --- |
| Driver HP60C | RGB và depth 640×480, FPS chung = 20 |
| Video LAN | 320×240 @ mục tiêu 20 FPS, JPEG quality 50 |
| Tuổi ảnh tối đa | 350 ms từ lúc ROS subscriber nhận tới lúc chọn ảnh để gửi |
| Hàng chờ | ROS depth=1, chỉ giữ sample mới nhất, TCP send buffer yêu cầu 16 KiB |
| Ảnh MQTT | Giữ nguyên FPS/kích thước/chất lượng đã cấu hình |

20 FPS có khoảng cách khung hình 50 ms; 15 FPS khoảng 66,7 ms. Nâng FPS có
thể giúp chuyển động đều hơn, nhưng không tự giải quyết độ trễ Wi-Fi, USB,
phơi sáng hoặc trình duyệt. Giới hạn tuổi ảnh không phải cam kết độ trễ
từ cảnh thật đến màn hình dưới 350 ms. Bộ đệm TCP thực tế còn phụ thuộc OS.

Code hỗ trợ thử 15 hoặc 20 FPS cho HP60C. Không đặt 30 FPS chỉ vì giới hạn
chung của streamer cho phép tới 30. Chưa thay phơi sáng, gain, độ sâu hay
point-cloud của driver ngoài FPS/kích thước nêu trên.

## Áp dụng sau khi cập nhật đầy đủ source lên Pi

Đợt này sửa source trong repo, chưa upload hoặc chạy thử trên Pi. Cần cập nhật
runtime đầy đủ tương ứng; không chép riêng một file `worker_config.py` từ repo
này lên một bản cũ thiếu module phụ thuộc. Giữ cấu hình riêng, token, CA,
hardware profile, journal và map hiện có.

Kết thúc nhiệm vụ qua trang điều khiển nếu còn đang chạy. Ở terminal
`robot.zsh run`, bấm `Ctrl+C` và đợi toàn bộ các dòng `STOP`.

Trong container MentorPi, dùng user ubuntu và shell ROS đã source:

```sh
cd /home/ubuntu/mediassist-mqtt-2026-10-08

# Chỉ kiểm tra: không ghi file, tự chọn đúng source/install theo need_compile.
pi/.venv/bin/python tools/configure_camera.py \
  --config pi/config/mqtt.worker.json --fps 20 --width 320

# Áp dụng sau khi kiểm tra thành công và stack đã dừng.
pi/.venv/bin/python tools/configure_camera.py \
  --config pi/config/mqtt.worker.json --fps 20 --width 320 --apply

zsh tools/robot.zsh run
```

Công cụ kiểm tra đúng node/package/parameter của ascamera trước khi ghi.
Nếu launch dùng cấu trúc khác, nó dừng và báo `CAMERA_LAUNCH_NOT_RECOGNIZED`.
Không đoán file hoặc khởi động thêm camera driver. `--vendor-launch` chỉ dùng
khi đã xác minh đường dẫn đó là launch đang được robot sử dụng.

Sao lưu nằm tại `pi/state/camera-backups/<timestamp>/`, gồm cấu hình worker,
launch driver trước sửa và manifest đường dẫn/hash. Lỗi ghi giữa chừng được
rollback; lỗi mất điện/crash hệ điều hành vẫn cần kiểm tra từ bản sao lưu.

## Kiểm tra đúng thông số thực nhận

Giữ terminal stack mở. Dùng một tab video LAN; không mở nhiều tab stream
trong lần đo đầu. Ở terminal container khác:

```sh
cd /home/ubuntu/mediassist-mqtt-2026-10-08
tail -n 35 pi/state/stack/camera.log
grep -E 'get set_fps|set rgb resolution|set depth resolution' pi/state/stack/slam.log | tail -n 6

ros2 topic echo /ascamera/camera_publisher/rgb0/image \
  --once --no-arr --qos-profile sensor_data

# Chạy khoảng 15 giây rồi Ctrl+C; đây là tốc độ nhận của subscriber đo.
ros2 topic hz /ascamera/camera_publisher/rgb0/image --window 100

curl --max-time 5 -s http://127.0.0.1:8081/health
```

Nếu log mới vẫn báo 15 FPS thì việc đổi nguồn chưa có hiệu lực; chưa được
kết luận đang chạy nguồn 20 FPS. Cần đối chiếu node/launch đang hoạt động.

Trang điều khiển dùng nút **Xem video trực tiếp**, URL (thay `IP_PI_HIEN_TAI`):

```text
http://IP_PI_HIEN_TAI:8081/stream?topic=/ascamera/camera_publisher/rgb0/image
```

Không cần thêm `fps`, `width` hoặc `quality` vào URL. Streamer này lấy thông
số từ file cấu hình, không áp dụng các query parameter đó.

### Đọc /health

| Field | Ý nghĩa |
| --- | --- |
| `targetFps` | Mục tiêu stream được cấu hình |
| `sourceReceivedFps` | Tốc độ ảnh hợp lệ subscriber stream nhận khi có người xem |
| `encodedFps` | Tốc độ ảnh được encode, cửa sổ khoảng 5 giây |
| `sourceSize` / `streamSize` | Kích thước nguồn / ảnh JPEG xuất ra |
| `lastEncodeMs` | Thời gian xử lý/resize/JPEG gần nhất |
| `lastReceiveToEncodeMs` | Từ callback ROS nhận sample đến khi JPEG được publish |
| `lastClientWriteMs` | Thời gian ghi JPEG vào socket của lần gửi gần nhất |
| `lastJpegBytes` | Dung lượng JPEG gần nhất |
| `frameAgeSeconds` / `frameFresh` | Tuổi sample từ lúc nhận tại ROS / còn đủ mới không |

Các số đo này không bao gồm toàn bộ thời gian phơi sáng/driver/USB trước
callback, cũng không đo lúc trình duyệt thực sự vẽ xong. Socket ghi nhanh
không chứng minh đường truyền và trình duyệt không có độ trễ. Khi không có
người xem, bộ encode nghỉ nên FPS đo có thể về 0.

## So sánh để chọn cấu hình

1. Đo 20 FPS ở 320×240 khi robot đứng yên, rồi khi SLAM đang quét. Cho người
   đi ngang khung hình với ánh sáng tương đương.
2. Nếu cả nguồn và encode ổn định gần 20, thử ảnh nét hơn: dừng stack,
   chạy lại công cụ với `--fps 20 --width 640 --apply`, rồi khởi động lại.
3. Nếu nguồn gần 20 nhưng encode thấp, giữ 320×240 và đối chiếu tải Pi.
4. Nếu cả hai gần 20 nhưng hình vẫn đi sau người thật, đo độ trễ hiển thị:
   quay điện thoại sao cho thấy đồng thời người thật/đồng hồ và màn hình
   laptop. Cách này giúp phân biệt thiếu FPS, nhòe chuyển động và trễ truyền.
5. Để so sánh với 15 FPS: dừng stack, dùng `--fps 15 --width 320 --apply`,
   khởi động lại và lặp phép đo. Đây là đổi chế độ so sánh, không phải phục
   hồi nguyên byte tất cả file từ bản backup.

Chưa kết luận cấu hình nào mượt nhất trên phần cứng cho tới khi có số đo
thực tế. Ưu tiên cấu hình giữ FPS ổn định và độ trễ thấp khi SLAM hoạt động.

## Xác nhận bản sửa trên laptop

- Bộ unittest: **189 bài**, **185 pass**, **4 skip**, không có bài fail.
- Đã thử patch FPS với file launch Hiwonder thật trong bản source lưu trên
  laptop; xác nhận chỉ thay parameter dự kiến và gọi lại không đổi thêm.
- Test cấu hình kiểm tra cả chế độ chỉ đọc, bảo toàn hardware/MQTT, từ chối
  stack đang chạy, launch không nhận diện được và rollback khi ghi file lỗi.
- Chưa đo FPS hoặc độ trễ màn hình trên Pi. Codec OpenCV/NumPy là một trong
  các bài skip do môi trường kiểm thử laptop thiếu thư viện đó.
