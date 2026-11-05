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

Nếu Docker khôi phục MentorPi trước khi Wi-Fi có DNS, cài bước sửa tự động
trên **Pi host** bằng `tools/install_container_dns_sync.py --apply` (sudo).
Đặt công cụ này cùng thư mục với `repair_pi_container_dns.py` trước khi chạy.
Service `mar-container-dns.service` chạy một lần sau Docker/network khi boot;
NetworkManager gọi lại khi kết nối hoặc DNS thay đổi. Không có vòng lặp nền,
không khởi động ROS/worker, không cần restart Docker hay robot để cài.

```sh
sudo systemctl start mar-container-dns.service
sudo journalctl -u mar-container-dns.service -n 25 --no-pager
```

Để gỡ cơ chế tự đồng bộ (không xóa bản sao DNS đã sửa):

```sh
sudo systemctl disable mar-container-dns.service
sudo rm /etc/NetworkManager/dispatcher.d/90-mar-container-dns
```

DNS không sửa được URL HTTPS cũ: khi IP laptop BE thay đổi, URL TTS/upload
trên Pi phải trỏ tới địa chỉ Pi truy cập được, chứng chỉ server phải có tên/IP
đó trong SAN, và CA tin cậy trên Pi phải khớp CA ký chứng chỉ. Device token
là thông tin xác thực API, không phải CA. Giữ kiểm tra TLS hoạt động.

## Thùng gắn cứng phía sau khi MOVE trên map đã lưu

Quét map giữ thân robot trần. Thùng gắn **cứng, thẳng và ở giữa** được cấu hình
riêng cho navigation; không dùng cấu hình này cho xe kéo có khớp quay. Đo phần
nhô thêm từ mép sau robot, kể cả giá đỡ. Sau khi dừng stack, trong container:

```sh
cd /home/ubuntu/mediassist-mqtt-2026-10-08
pi/.venv/bin/python tools/configure_navigation_geometry.py \
  --rear-extension .10 --cargo-width .15
pi/.venv/bin/python tools/configure_navigation_geometry.py \
  --rear-extension .10 --cargo-width .15 --apply
```

Với biên trần front=.109, rear=.104, left/right=.086 m từ `base_footprint`,
thân cộng thùng có rear=.204 m, tổng dài .313 m và rộng .172 m. Hình chữ nhật
bao dùng chiều rộng robot cả phần đuôi (dư 1,1 cm mỗi bên so với thùng 15 cm).
Padding Nav2 hiện có vẫn tính riêng, không cộng 10 cm vào cả trước và sau.

Lệnh cấu hình lưu mặc định `cargo` cho **những phiên saved-map mới**; không
đổi footprint/tốc độ quét map, hardware profile gốc, URL, TLS hay asset map.
Chạy navigation từ cấu hình worker gốc; không đưa cấu hình phiên đã sinh làm
đầu vào để cộng chiều dài thêm lần nữa:

```sh
zsh tools/navigation_session.zsh --session-id SESSION_ID --load-profile cargo
# Khi tháo thùng và vẫn muốn MOVE trên map đã lưu:
zsh tools/navigation_session.zsh --session-id SESSION_ID --load-profile bare
# Quét map mới không gắn thùng:
zsh tools/robot.zsh run
```

Mỗi phiên cargo tạo bản config/hardware tạm với cùng biên cho localization,
endpoint, zone, phanh LiDAR và hai costmap. Cargo chọn Smac Hybrid SE2 để
planner xét hình xe và hướng thay vì Smac 2D; RPP vẫn theo đường tiến và xử lý
hướng cuối như trước. Do đó có thể chọn đường khác hoặc từ chối lối rẽ mà
robot trần đi lọt. Không thay TF/odom hay tọa độ endpoint; không cần quét lại
map chỉ vì đổi footprint. Chọn profile trước khi khởi chạy, không thay giữa
job đang chạy. Thử di chuyển/quay chậm với thùng để xác nhận kích thước đo và
LiDAR không bị thùng che/nhìn vào chính thùng.

Cargo MOVE có một lần lùi thoát tối đa 10 cm, tốc độ 0,05 m/s khi cần quay.
Trước khi chạy, Pi kiểm tra cả thân/thùng theo scan, map và vùng cấm, rồi thử
tính đường từ vị trí sau khi xoay/lùi. Không có đường hoặc phía sau bị chặn
thì không tự lùi. Khi tới endpoint, chỉ lùi nếu vẫn nằm trong ngưỡng đến đích
10 cm, rồi gửi một Spin theo hướng endpoint. Quét map và MOVE không thùng
không tự dùng bước thoát này. Gate xét cung quay thực tế có khoảng phanh,
không mặc định chặn vì có vật nằm trong vòng tròn quay 360 độ.

Khi thùng không quay được theo cung ngắn, Pi kiểm tra cả cung ngược trước.
Nếu cần, MOVE cargo có thể tiến/lùi hoặc dịch ngang 6–10 cm ở 0,05 m/s rồi
tính lại path từ vị trí thật. Đây là thao tác thoát cục bộ; RPP vẫn ưu tiên
đi tiến/cua khi dẫn đường. Chỉ thực hiện sau khi scan, map, vùng cấm và đường
từ vị trí dự kiến đều hợp lệ. Nếu đang chỉ chạm phần đệm, chỉ cho tịnh tiến
rời xa mọi tiếp xúc ban đầu, không chạm thân thật, không thêm tiếp xúc khác
và phải thoát hết đệm trước khi quay. Không bỏ kiểm tra unknown hoặc vùng cấm.
Gate chặn bánh liên tục một giây sẽ dừng goal để chọn thao tác khác, thay vì
chờ hết các khoảng kiểm tra tiến triển 20 giây. Không lặp vô hạn trong một step.

Đọc footprint của runtime đang chạy (thay đường dẫn theo phiên):

```sh
pi/.venv/bin/python tools/check_robot_geometry.py \
  --config pi/state/navigation-sessions/SESSION_ID/RUN_ID/worker.json --config-only
```

Đổi mặc định về không thùng bằng cùng lệnh cấu hình với
`--default-profile bare --apply`. Bản cấu hình cũ được sao lưu khi apply.

MOVE giữ path Nav2 khi đích không đổi và phần đường còn lại vẫn hợp lệ.
Kiểm tra va chạm đường ở 2 Hz dùng costmap trên Pi, không gọi BE/DB để xin
quyền cho từng điểm. Chỉ tính lại khi path bị chắn hoặc đổi đích; kiểm tra
LiDAR/gate vẫn chạy nhanh hơn để phanh. Cargo ưu tiên hướng tiếp cận thẳng
tới đích, sau đó mới xử lý hướng endpoint bằng một Spin. Nếu hướng tiếp cận
đó không lọt, planner thử hướng đã lưu trước khi báo không có đường.
RPP nhìn trước 0,28–0,50 m (0,9 s), gia tốc quay 1,8 rad/s² để giảm dao động
lái trên đoạn thoáng. Các giá trị được tạo trong `nav2_params` ở
`pi/mediassist/navigation_setup.py`; không sửa file runtime của phiên cũ.
Nếu Nav2 đã hết các lượt tránh vật cản trong BT và trả NAV_ABORTED, worker
cho tối đa hai lượt dừng/tính đường lại tới cùng endpoint trong cùng step,
và một lượt hiệu chỉnh nếu sai lệch vị trí cuối nhỏ hơn hoặc bằng 25 cm. Lùi thoát
cargo vẫn tối đa một lần và phải có đường hợp lệ từ vị trí dự kiến. Lỗi định
vị, hủy lệnh hoặc chưa xác nhận dừng không được tự thử lại theo nhánh này.

## Thử micro/loa và STT/TTS

Thử micro/loa và STT/TTS trong container MentorPi, không cần chạy stack ROS:

```sh
cd /home/ubuntu/mediassist-mqtt-2026-10-08
pi/.venv/bin/python tools/voice_audio.py devices
pi/.venv/bin/python tools/voice_audio.py status
pi/.venv/bin/python tools/voice_audio.py record --seconds 5 --play
pi/.venv/bin/python tools/voice_audio.py tts --text 'Xin chào, tôi là robot MediAssist.' --play
pi/.venv/bin/python tools/voice_audio.py roundtrip --seconds 5 --play
```

Thu âm qua micro USB, phát qua loa USB; `auto` yêu cầu đúng một card USB.
Nếu có nhiều card, truyền `--input-device` / `--output-device` bằng tên ALSA
như `plughw:CARD=Device,DEV=0`. `record` chỉ thử phần cứng; `roundtrip` gửi WAV
PCM16 mono 16 kHz tới BE `/api/v1/pi/speech/transcribe`, hiện transcript rồi
gọi `/api/v1/pi/speech/synthesize` để phát lại. Không dispatch ý định/MOVE.
Token được lấy từ cấu hình thiết bị, HTTPS/CA được kiểm tra; không đưa API key
AI lên Pi. File thu tạm được xóa, chỉ lưu WAV nếu có `--output /tmp/name.wav`.
BE cần bật `STT_ENABLED=true`, `TTS_ENABLED=true` và cấu hình API key riêng.

Phát loa như một bước nhiệm vụ dùng `INTERACT/SPEAK`. Trên trang mapping,
thêm `SPEAK` vào workflow test qua `POST /api/v1/test-workflows/run`, hoặc dùng
`POST /api/v1/speech-jobs` với workflow SPEECH đã publish. Các API job này cần
worker/MQTT online; khác với lệnh chẩn đoán thu/phát trực tiếp phía trên.

### Thu âm từ xa qua API job

Trong Swagger, đăng nhập ADMIN rồi gọi:

```http
POST /api/v1/robots/{robotId}/voice-test
Idempotency-Key: <UUID mới cho mỗi lần thu>
Content-Type: application/json

{"durationSec":5,"languageCode":"vi","playback":true,"volume":70}
```

BE trả `jobId` ngay. Khi worker/MQTT online, Pi thực thi
`EXECUTE_SENSING/VOICE_CAPTURE`: thu micro PCM16 mono 16 kHz trong 1..15 giây,
gửi WAV qua HTTPS cho STT và trả transcript qua kết quả step. Nếu `playback=true`,
step `INTERACT/SPEAK` tiếp theo đọc lại transcript (tối đa 500 ký tự).
`GET /api/v1/jobs/{jobId}`: đọc `steps[].resultJson.metrics.text` tại step thu âm.
`POST /api/v1/jobs/{jobId}/cancel` hủy qua runtime thông thường; arecord được dừng
và thu hồi tiến trình. File thu tạm bị xóa; text được lưu trong kết quả job.
Không cần map hoặc khởi tạo AMCL cho job chỉ thu/phát; không tự khởi động micro
khi run và không chuyển transcript thành lệnh di chuyển. Trong workflow test,
có thể dùng `VOICE_CAPTURE` rồi `SPEAK_LAST_TRANSCRIPT` bên cạnh các step đã có.

### Nút thử WAV gốc trên trang HTML

Trang `mapping.html` có nút **Thu và phát lại WAV gốc trên robot**. Nút gọi
`POST /api/v1/robots/{robotId}/audio-loopback-test` với
`{"durationSec":5,"volume":70}` và Idempotency-Key mới cho mỗi lượt.
Pi bíp báo sẵn sàng, thu PCM16 mono 16 kHz rồi phát lại bản ghi; không gọi STT
hoặc TTS. Nói sau tiếng bíp. File tạm bị xóa khi hoàn tất/hủy; kết quả job chỉ
lưu thời lượng, định dạng, mức tín hiệu và trạng thái phát, không lưu âm thanh.
Giữ stack/worker chạy và chờ MQTT sẵn sàng; không cần phiên map cho job chỉ thử loa.
Nút **Chạy một job phát nội dung trên** dùng `INTERACT/SPEAK`; cũng có thể thêm
step phát loa vào workflow MOVE/WAIT/servo bằng nút **Thêm phát loa vào workflow**.

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
