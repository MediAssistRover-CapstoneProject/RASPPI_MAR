# Khung xác thực khuôn mặt trên màn robot

Trang cục bộ `http://127.0.0.1:8096/` hiển thị video và khung khi BE mở bước
`FACE_VERIFICATION`. Chạy trang này bằng trình duyệt trên màn hình gắn vào Pi.
Trang không mở phiên, không chọn staff và không điều khiển servo.

## Luồng

1. `robot_stack.py run --mode navigation` khởi động camera, worker và
   `face_guide.py` nếu `staff_face_enabled`, camera và stream đều bật.
2. Worker chỉ lấy frame trong cửa sổ face do BE cấp qua MQTT, khi robot đứng yên.
   Worker ghi `pi/state/stack/face-guide.json` gồm mã trạng thái và thời hạn,
   không ghi ảnh, ID staff, job, token hoặc mẫu khuôn mặt.
3. Trang loopback đọc `/status` mỗi 0,5 giây, chỉ mở stream camera khi cửa sổ
   đang hoạt động. Không có phiên, hết hạn hoặc mất worker thì ẩn ảnh.
4. AI_FACE kiểm tra mặt nằm trọn trong vùng chuẩn hóa `x=20–80%`, `y=12–88%`
   của ảnh **verify** trước khi so avatar. Avatar enrollment không dùng vùng này.
   Trang vẽ đúng vùng đó. Camera có thể là web_video_server 8080 hoặc Python
   camera_stream 8081; không thêm bộ đọc camera phần cứng thứ hai.
5. Mặt ngoài khung trả `FACE_OUTSIDE_FRAME`; BE không cấp quyền. Mặt khớp thì
   BE tiếp tục workflow AUTH → UNLOCK với grant có hạn như trước.

## Đưa lên robot

- Cập nhật **AI_FACE và BE trước Pi** để `FACE_OUTSIDE_FRAME` đi hết đường về Pi.
- Giữ nguyên cấu hình camera, MQTT, servo và credential của Pi. Không thay
  `pi/config/mqtt.worker.json` bằng file mẫu từ Git.
- Trên màn gắn với Pi, mở trình duyệt toàn màn hình tại
  `http://127.0.0.1:8096/`. Trang chỉ nghe trên loopback của Pi, nên không
  truy cập được từ máy khác trong LAN.
- Kiểm tra `pi/state/stack/face-guide.log` và `worker.log` nếu trang không hiện.
  Trang này cần một màn hình và trình duyệt có sẵn trên Pi; repo chưa cấu hình
  trình duyệt tự mở khi Pi bật nguồn vì phần cứng màn hình chưa được xác định.

## Kiểm tra an toàn

- Khi chưa có bước face: trang ẩn video và hiện trạng thái chờ.
- Khi có bước face: khung ở giữa ảnh; ngoài khung, nhiều mặt, thiếu mặt, ảnh
  mờ và sai người đều không mở ngăn.
- Khi xác thực thành công: trang hiện thông báo ngắn; lệnh mở vẫn phải được BE
  cấp và Pi xác nhận. Không diễn giải thông báo là servo đã mở thành công.
- Hủy job, chuyển bước, mất MQTT, hết hạn: trang trở về trạng thái chờ.
- Đo lại trên camera robot 640×480, 15 FPS; kiểm tra tỷ lệ hiển thị không bị
  lật gương hoặc cắt hình. Bản demo không chống ảnh/video giả.
