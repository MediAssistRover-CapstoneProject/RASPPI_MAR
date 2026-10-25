# Tự khám phá map và tự dừng — MAR Frontier Mapping, 23/10/2026

## Đã làm và giới hạn thực tế

- Pi dùng OccupancyGrid thật của SLAM để tìm frontier: biên giữa ô đã biết trống và ô chưa biết/không chắc chắn. Màu đen (occupied) không phải phần phải quét cho biến mất.
- Planner thu nhỏ bản sao dùng để lập đường xuống ô >=10cm, tối đa 300×300 ô. Map SLAM, preview, PGM/YAML và endpoint giữ nguyên độ phân giải. Ô occupied >=65, free <=25, phần còn lại là unknown/uncertain, thống nhất ngưỡng lưu map.
- Tìm vùng free liên thông với robot, tách nhóm frontier, nở vật cản/unknown theo bao tròn chứa toàn thân robot/xe kéo + margin, tìm đường trên free đã biết, chọn điểm đứng lùi trước frontier. Không coi unknown là đường trống.
- Chạy planner trong thread nền để vòng MQTT/lease và dừng robot không chờ tìm đường. Planner quá hạn hoặc dữ liệu cũ thì dừng.
- Bộ điều khiển cục bộ theo đường, giảm tốc theo LiDAR, chọn hướng tránh vật cản. Có thể dừng khi không có đường/rẽ không đủ khoảng trống. Đây là planner frontier + điều khiển cục bộ trong chế độ mapping, không khởi động thêm Nav2 navigation hoặc chương trình tránh vật cản hãng song song.
- Xe kéo có khớp dùng đường cong tiến, giới hạn bán kính quay đã đo, không tự lùi/quay tại chỗ. Bao kích thước phải chứa mọi góc xoay của xe kéo. Đây chưa phải mô hình động học đầy đủ có cảm biến góc khớp hoặc chứng nhận tránh va chạm cơ học.
- Tùy chọn hỗ trợ manual dùng chung scan cho gamepad/UI: giảm tốc/dừng khi gần vật cản, không tự bẻ hướng khi người dùng đang điều khiển. Bộ ramp vận tốc, driver chassis hãng, gate và watchdog hiện có vẫn dùng chung.
- Camera hãng 8080 và lựa chọn quay lại Python 8081 được gộp trong bộ vá này.

**Chưa cài lên Pi, chưa bật tự chạy thực tế.** Mặc định profile cũ vẫn là `reactive`; cần chọn `frontier` khi cấu hình. Bộ vá không tự thay các số đo cơ khí hoặc tự tạo job di chuyển.

## Khi nào tự dừng?

Các mặc định trong profile mới:

1. Không còn frontier đáng kể trong vùng free liên thông mà robot đang quét. Cụm frontier ngắn hơn 0,20m bị lọc như nhiễu nhỏ và được báo số ô bị lọc.
2. Kết quả ổn định ít nhất 15 giây, tối thiểu ba bản map riêng biệt, đã chạy ít nhất 30 giây, vùng free quan sát được >=0,5m².
3. Scan, pose, phiên điều khiển còn hợp lệ; map mới nhất phải khớp nội dung được dùng để xác nhận hoàn tất.
4. Pi phát zero velocity, ngừng lease chuyển động, rồi báo:

```json
{
  "state": "STOPPED",
  "reason": "COVERAGE_COMPLETE",
  "finished": true,
  "coverage_complete": true,
  "exploration_mode": "frontier",
  "exploration": {
    "enabled": true,
    "state": "COMPLETE",
    "frontier_count": 0
  }
}
```

Ý nghĩa là đã hoàn tất theo tiêu chí frontier của vùng đang tiếp cận được; không chứng minh đã thấy mọi mặt sau đồ vật hoặc phòng nằm sau cửa đóng. Tường đen và xám ngoài tường vẫn giữ nguyên. Hết thời gian, mất lease, scan/pose lỗi, thiếu khoảng trống, frontier bị chặn hoặc không thêm thông tin đều **không** bật `coverage_complete`.

HTML hiển thị số frontier và trạng thái. Với profile frontier, tùy chọn “dừng khi về gần điểm đầu” bị tắt để không kết thúc sớm chỉ vì đi vòng. Khi chọn checkbox lưu tự động, HTML gửi event `MAPPING_SAVE_REQUESTED` trên cùng job sau khi nhận hoàn tất hợp lệ. Luồng SAVE → upload → COMMIT → STOP hiện tại tiếp tục xử lý; phải chờ job SUCCEEDED để xác nhận đã lưu trên BE.

Trình duyệt vẫn phải mở và gia hạn quyền tự chạy như luồng cũ. Đóng trang/mất kết nối làm robot dừng, không tự khởi động lại nhiệm vụ.

## Thứ tự cài: BE trước, Pi sau

Heartbeat Pi có thêm trạng thái frontier. BE cũ có schema chỉ chấp nhận `coverage_complete=false`, do đó phải cập nhật BE trước để tránh heartbeat bị từ chối. BE mới vẫn chấp nhận Pi/profile cũ.

Source BE đã được cập nhật tại:

```text
C:\Users\LENOVO\Downloads\MAR_EMQX_Demo\BE_MAR
```

Các file thay đổi cũng được đồng bộ sang repo GitHub BE_MAR trên OneDrive và nằm trong thư mục `be/` của bộ đầy đủ. Không thay database, biến môi trường, TLS, token hoặc workflow đã lưu.

Hoàn tất lưu/dừng job đang chạy rồi dừng stack Pi bằng Ctrl+C, chờ STOP. Dừng server video hãng mở tay bằng Ctrl+C trước khi cấu hình backend 8080.

### Windows: build/restart BE từ source đã cập nhật

Chỉ chạy sau khi không còn nhiệm vụ cần giữ đang thực hiện; thao tác này restart backend:

```powershell
Set-Location 'C:\Users\LENOVO\Downloads\MAR_EMQX_Demo\BE_MAR'
docker compose -p mar-emqx-demo up -d --build backend
```

Chờ backend khởi động thành công, kiểm tra:

```powershell
curl.exe --fail http://127.0.0.1:8080/actuator/health
docker compose -p mar-emqx-demo exec -T reverse-proxy nginx -s reload
Start-Process 'http://127.0.0.1:8080/mapping.html#mappingAutonomy'
```

Nếu health chưa thành công, đọc `docker compose -p mar-emqx-demo logs --tail 80 backend`, chưa bật tự chạy Pi.

### Windows: upload bộ Pi

```powershell
scp 'C:\Users\LENOVO\Downloads\MAR_FrontierMapping_PI_2026_10_23.zip' pi@192.168.1.22:/tmp/
ssh pi@192.168.1.22
```

### HOST Pi

```sh
sudo docker cp /tmp/MAR_FrontierMapping_PI_2026_10_23.zip MentorPi:/tmp/MAR_FrontierMapping_PI_2026_10_23.zip
sudo docker exec -it -u ubuntu -w /home/ubuntu/mediassist-mqtt-2026-10-08 MentorPi zsh
```

### Container MentorPi

```sh
mkdir /tmp/mar-frontier-20261023
pi/.venv/bin/python -m zipfile -e /tmp/MAR_FrontierMapping_PI_2026_10_23.zip /tmp/mar-frontier-20261023
pi/.venv/bin/python /tmp/mar-frontier-20261023/install_pi_patch.py
pi/.venv/bin/python /tmp/mar-frontier-20261023/install_pi_patch.py --apply
```

Nếu `mkdir` báo đã tồn tại, dùng tên thư mục mới. Nếu installer báo `CODE_BASELINE_MISMATCH`, gửi tên file và giữ nguyên code; không bỏ qua kiểm tra. Bộ cài chỉ ghi các file code trong manifest và lưu backup trước khi ghi. Config/maps/secrets/journal không nằm trong payload.

## Cấu hình cơ khí trước khi bật tự chạy

Xe người dùng là **xe kéo có khớp**. Những số đo ước lượng thùng 150mm và khớp 20–30mm trước đó chưa đủ để khẳng định vùng quét khi rẽ. Cần bốn khoảng từ tâm `base_footprint` tới biên bao toàn bộ xe kéo ở mọi góc xoay, và bán kính quay tiến nhỏ nhất đã kiểm tra với xe thật.

Lệnh dưới là mẫu cho số đo **thực**, không copy nguyên với chữ HOA:

```sh
pi/.venv/bin/python tools/configure_autonomous_mapping.py --enable --vehicle articulated --front FRONT_M --rear REAR_M --left LEFT_M --right RIGHT_M --turn-radius TURN_RADIUS_M --speed 0.06 --angular 0.15 --duration 300 --exploration frontier --manual-lidar-assist
```

Đọc plan. Khi các số đo đúng và plan hợp lệ, chạy lại cùng lệnh thêm `--apply`. Với xe kéo đang gắn, không chọn `--vehicle rigid` để vượt kiểm tra hình học. Tốc độ thực còn bị giới hạn bởi hardware profile hiện có.

Không gắn xe kéo thì cấu hình `--vehicle rigid` và dùng kích thước thân thực. Body 212×171mm chưa tự xác nhận vị trí gốc base_footprint hoặc đồ gắn thêm; phải đối chiếu trước khi dùng.

Nếu muốn giữ manual hoàn toàn như cũ, bỏ tùy chọn `--manual-lidar-assist`. Nếu bật, trước nguy cơ va chạm nó giảm tốc/dừng input tay, không tự lái vòng qua đồ vật.

### Camera hãng (tùy chọn, sau khi stack và server tay đã dừng)

```sh
pi/.venv/bin/python tools/configure_camera_backend.py --backend web_video_server --apply
```

### Chạy robot và test

```sh
zsh tools/robot.zsh run
```

Giữ terminal mở, tạo đúng một mapping job bằng trang HTML hiện có. Chờ map/pose mới, kiểm tra report có `exploration_mode: frontier`, rồi bấm “Bắt đầu tự quét”. Dùng giới hạn 300 giây hoặc thấp hơn profile. Checkbox tự lưu là tùy chọn. Nút STOP/gamepad override vẫn ưu tiên dừng.

Camera URL nếu dùng backend hãng:

```text
http://192.168.1.22:8080/stream?topic=/ascamera/camera_publisher/rgb0/image&type=mjpeg&width=640&height=480&quality=60
```

Test thực theo thứ tự: khu trống có biên rõ, thêm vật cản để quan sát giảm tốc/dừng, mở một vùng chưa biết để xem frontier tăng, chặn lối đi để xem báo BLOCKED chứ không COMPLETE, rồi để hoàn tất và kiểm tra job SUCCEEDED + PNG trên BE. LiDAR 2D không thấy mọi vật quá thấp/cao, mặt kính hoặc phần xe kéo tự che; cần kiểm thử đúng môi trường và phần cứng thật.

## Quay về chế độ cũ

Khi stack đã dừng, chạy lại helper với đúng hình học đã đo và `--exploration reactive`; bỏ `--manual-lidar-assist` để tắt hỗ trợ manual. Hoặc `--disable --apply` để tắt tự chạy, giữ mapping/manual hiện có:

```sh
pi/.venv/bin/python tools/configure_autonomous_mapping.py --disable --apply
```

Muốn đổi camera về 8081: `pi/.venv/bin/python tools/configure_camera_backend.py --backend python --apply`. Backup code ở `pi/state/code-backups`; dùng `--rollback` của installer khi stack đã dừng. Giữ BE mới tương thích trước khi rollback Pi. Không xóa journal để hủy job.

## Kiểm thử đã hoàn tất

- 111 kiểm thử Pi: frontier/path, phòng khép kín, vùng chưa biết/uncertain, cửa/hành lang bị chặn, origin xoay, xác nhận ổn định, source map thật, lease/scan lỗi, vật cản mới, manual assist, trailer radius, gamepad, SLAM, camera và supervisor.
- Giả lập robot tiến trong phòng thẳng, mở thêm map theo vị trí và dừng khi frontier hết; không thay thế test cơ khí thật.
- 68 ca BE unit/wire qua, một ca cũ đang được skip.
- 31 ca MySQL integration qua trên database tách riêng, gồm coverage heartbeat → API → SAVE/upload/COMMIT/STOP → PNG.
- Sáu ca JS và kiểm thử Chrome headless trên HTML nhúng thật: không tự chạy khi tải trang, đúng quyền điều khiển/phiên, tự lưu một lần khi opt-in, không tự lưu khi planner lỗi.
- Bộ cài kiểm tra Python 3.10, baseline, backup và rollback trước khi phát hành ZIP.

Code đã cập nhật trên laptop; bản này chưa được bật hoặc kiểm thử chuyển động thực tế trên Pi, chưa restart BE đang chạy.
