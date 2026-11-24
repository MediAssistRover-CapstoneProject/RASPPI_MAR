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

## Quan sát té ngã tại checkpoint hoặc trong MOVE/IDLE

Trong `pi/config/mqtt.worker.json`, thêm vào mục `ai` cấu hình sau. File token
nằm ngoài repo, chỉ cho user chạy worker đọc.

```json
{
  "patrol_fall": {
    "enabled": false,
    "mode": "CONTINUOUS",
    "base_url": "https://media.mediassistrover.duckdns.org",
    "token_file": "/home/ubuntu/mar-secrets/fall-ai.token",
    "rate_hz": 4,
    "min_available_memory_mb": 512,
    "cooldown_sec": 60,
    "depth": {
      "enabled": false,
      "registered_depth_verified": true,
      "depth_topic": "/ascamera/camera_publisher/depth0/image_raw",
      "rgb_info_topic": "/ascamera/camera_publisher/rgb0/camera_info",
      "depth_info_topic": "/ascamera/camera_publisher/depth0/camera_info",
      "rgb_rectified": false,
      "floor_z_m": 0,
      "near_floor_max_height_m": 0.45
    }
  }
}
```

Khi BE đã nhận workflow tương ứng, mở phiên navigation và thêm `--patrol-fall`
vào lệnh chạy phiên để bật cho lần chạy đó. Ở BE, dùng checkpoint
`sensingType: "PATROL_FALL"` để quan sát riêng, hoặc `fallDetection: true`
để thêm vào các checkpoint quan sát vùng đã có. Dùng `sensingWindowSec: 12`
cho lần thử đầu. `PATROL_FALL` riêng không cần cấu hình bộ nhận diện vùng cấm.

Để giám sát nền trong MOVE/IDLE, dùng `--fall-monitor` thay cho `--patrol-fall`.
Task này không đổi trạng thái robot, không trực tiếp điều khiển bánh và không
đưa lỗi AI thành lỗi MOVE. Nó dùng chung ROS camera, giữ một ảnh RGB mới nhất,
tối đa sáu ảnh depth và chỉ một request AI đang chạy. QR được ưu tiên;
thiếu RAM khả dụng sẽ tạm ngưng lấy ảnh AI. Khi SOS giữ micro/loa, phân tích RGB
vẫn chạy để hỗ trợ staff quan sát; không phát thêm cảnh báo lặp trong cuộc gọi.
Thời gian cooldown chỉ hạn chế tạo sự kiện mới, không làm đứng khung AI. Sự kiện dùng
`POST /api/v1/pi/fall-events`, độc lập với job/step. Phải deploy BE/AI tương ứng
và cài token trước khi bật; cấu hình mặc định giữ tắt cả detector và depth.

### Khung nhận diện trên video WebRTC

AI trả `overlay` từ cùng lần suy luận pose. Worker công bố metadata tại ROS
`/mediassist/ai/overlay` (giữ bản mới nhất); media bridge gửi gói lossy
`mar.ai.overlay.v1` trong đúng phiên LiveKit. Pi không vẽ/encode lại video và
không chạy thêm model. Task metadata có timeout 200 ms, không nằm trong vòng
MOVE hoặc vòng phát video; không có hàng đợi gửi lại các khung cũ.

Gói gồm `v`, `media_session_id`, `seq`, `captured_at`, kích thước ảnh,
`status` và `detections`. Gói tối đa 1200 byte; khi quá giới hạn ưu tiên giữ
khung nghi té ngã rồi khung người có độ tin cậy cao. Timestamp cũ hơn 1,5 giây
bị loại. UI kiểm tra participant robot/session/sequence, vẽ canvas đúng vùng
video và xóa khung khi rỗng, mất video, reconnect hoặc hết TTL. Đây là overlay
theo ảnh AI gần nhất, có nhãn tuổi kết quả; không trì hoãn video để đồng bộ
từng frame. Một khung đỏ là nghi vấn để staff xem, không tự xác nhận sự cố.

Triển khai AI service có phản hồi `overlay` và BE có quyền data chỉ cho robot
trước, sau đó cài Pi `media_runtime.py`, `media_bridge.py`, `patrol_fall.py`.
Trang `/media-call.html` đi kèm BE mới. Dùng `--fall-monitor` cho MOVE/IDLE;
`--patrol-fall` chỉ phân tích trong cửa sổ sensing của checkpoint. Chỉ chạy
`robot.zsh run` để quét map không tự tạo observer navigation.

Nhánh `PERSON_ON_FLOOR` nhận người đã nằm sẵn bằng tư thế ngang duy trì ít
nhất hai giây ở service. Pi chỉ gửi cảnh báo nhánh này khi RGB/depth cùng khung,
TF hợp lệ và cả hai hông không cao hơn ngưỡng `near_floor_max_height_m` so với
sàn. Đây là nghi vấn cần staff xem camera, không phải kết luận té ngã. Khi
camera đang di chuyển, không suy luận quá trình ngã từ vận tốc hông trên ảnh;
nhánh người nằm sẵn vẫn có thể quan sát nếu đủ depth/TF. Thời điểm chụp và cờ
chuyển động được gửi tới AI để tránh tính vận tốc từ độ trễ mạng.

Pi lấy khung RGB từ cùng topic của camera đang chạy và chỉ gửi trong lúc đứng
yên tại checkpoint. Khi nghi té ngã, worker gửi sự kiện cho BE để staff trực
xác minh. Staff nhận phiên `MONITOR` trên `/media-call.html`, xem camera trực
tiếp rồi bấm xác nhận. BE dùng bản đồ đã lưu để tính đường từ endpoint phòng
SOS tới điểm người đã đo được bằng RGB-D, nếu có tọa độ đáng tin cậy và lối đi
hợp lệ. Response chứa riêng điểm người, vị trí robot lúc quan sát và endpoint
gần người có thể đề xuất cho một lệnh MOVE. Khi thiếu depth/TF hoặc hông bị che,
BE chỉ có điểm robot lúc quan sát. Luồng tuần tra dừng ở checkpoint khi
báo nghi té ngã. SOS do người bấm dùng luồng SOS bình thường.

Độ sâu cần trùng frame và thông số camera RGB, lệch thời gian không quá 40 ms.
Pi lấy median depth quanh hai điểm cơ thể của đúng người kích hoạt cảnh báo.
Nếu có đủ hai hông, điểm map là trung điểm hông; nếu chỉ có hông và gối/cổ
chân, điểm map giữ tại hông. Pi dùng TF tại thời điểm chụp và gửi `patientLocation` có
map/hash/session/revision. Trường hợp nhiều người chồng lên nhau hoặc depth
không nhất quán sẽ bỏ tọa độ người. Trước khi bật, kiểm tra hướng và vị trí
mount camera ngoài robot để TF khớp thực tế. Cấu hình mẫu dành cho camera
AS đã xác minh registered depth; không áp dụng chỉ dựa vào tên topic.

AI v5 còn trả `floor_candidate` cho thân người hướng dọc camera, chưa coi là
té ngã. Pi chỉ chuyển thành nghi nằm trên sàn khi đứng yên, hông và gối/cổ
chân cùng người đều thấp tối đa 30 cm, chênh cao tối đa 18 cm và khoảng ngang
25–120 cm trong 3D. Có thể chỉnh `depth.end_on_max_height_m` trong 0,15–0,45 m;
không vượt ngưỡng chung `near_floor_max_height_m`. Không dùng điểm thấp nhất
toàn ảnh hoặc tự suy vị trí từ khung người khi thiếu keypoint. Log
`FALL_MONITOR_HEALTH` gồm `location_reason` và số pixel depth hợp lệ tại điểm
đo để phân biệt thiếu depth, hai điểm trùng nhau, sai calibration và thiếu TF.

## Thùng gắn cứng phía sau khi MOVE trên map đã lưu

Quét map giữ thân robot trần. Thùng gắn **cứng, thẳng và ở giữa** được cấu hình
riêng cho navigation; không dùng cấu hình này cho xe kéo có khớp quay. Đo phần
nhô thêm từ mép sau robot, kể cả giá đỡ. Sau khi dừng stack, trong container:

```sh
cd /home/ubuntu/mediassist-mqtt-2026-10-08
pi/.venv/bin/python tools/configure_navigation_geometry.py \
  --rear-extension .08 --cargo-width .15
pi/.venv/bin/python tools/configure_navigation_geometry.py \
  --rear-extension .08 --cargo-width .15 --apply
```

Với biên trần front=.109, rear=.104, left/right=.086 m từ `base_footprint`,
thân cộng thùng có rear=.184 m, tổng dài .293 m và rộng .172 m. Hình chữ nhật
bao dùng chiều rộng robot cả phần đuôi (dư 1,1 cm mỗi bên so với thùng 15 cm).
Nav2 tính riêng padding 2 cm ở mỗi cạnh, nên hình bao có đệm là 33,3 × 21,2 cm.

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

Phiên navigation mới mặc định `--travel-policy FORWARD_PATH`: robot quay theo
path rồi tiến/cua để dẫn đường. Đi ngang/lùi ngắn vẫn dành cho phục hồi đã
kiểm tra khoảng trống; quét map giữ cấu hình riêng. Chỉ chọn
`--travel-policy HOLONOMIC` khi muốn giữ hướng thân và dịch ngang khi đi bình
thường. Đổi chính sách cần mở lại phiên, không đổi controller giữa MOVE.

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

### Hội thoại RAG trên laptop

Chạy AI trên laptop bằng `run_robot_api.ps1` trong repo `AI_VOICE`. Dịch vụ
HTTPS cổng 7861 dùng index tài liệu của AI_VOICE; OpenAI key chỉ nằm ở server.
Pi dùng `/home/ubuntu/mar-secrets/voice-ai.json`, public CA và một token AI
riêng. Không chép OpenAI key, CA private key hoặc server private key lên Pi.

Trong terminal container MentorPi, chạy bằng user `ubuntu`:

```sh
cd /home/ubuntu/mediassist-mqtt-2026-10-08
pi/.venv/bin/python tools/voice_audio.py chat --start-now --once
```

Robot chào, phát bíp rồi mới nghe. Nói một câu; sau khoảng 0,8 giây im lặng,
Pi gửi WAV PCM16 mono 16 kHz tới BE STT, gửi text tới RAG laptop, rồi phát
câu trả lời bằng BE TTS. Nói “tạm biệt” hoặc Ctrl+C để kết thúc. Lệnh này không
khởi động ROS/động cơ/worker. Micro và loa có khóa chung với worker để tránh
hai tiến trình tranh thiết bị. Không gọi thử SPEAK/VOICE_CAPTURE từ job khi
đang trò chuyện vì chúng sẽ chờ chung thiết bị âm thanh.

Bỏ `--start-now` để chờ tín hiệu wake từ `/dev/wonderecho`. Cụm từ đánh thức
thực tế phụ thuộc firmware WonderEcho; cần thử “Hello MAR” trên thiết bị.
Nếu chưa phát ra frame wake, dùng `--start-now`; phần mềm này không nạp lại
firmware hoặc tuyên bố đã thay cụm từ đánh thức của hãng.

Âm thanh người dùng được đọc từ ALSA vào RAM, không ghi WAV thu âm vào thẻ,
không upload kho lưu âm thanh. Bộ đệm được bỏ sau STT; lịch sử chữ/câu trả lời
và nguồn tài liệu được ghi vào BE qua `/api/v1/pi/voice/*`. Pi chỉ giữ ID/
bộ đếm của phiên để đóng phiên bị gián đoạn, không giữ transcript trên đĩa.
File bíp và WAV TTS là âm thanh hệ thống, được xóa khi phát xong/hủy.

Để bật lệnh di chuyển, đặt `actions_enabled=true` trong file private
`/home/ubuntu/mar-secrets/voice-ai.json`; `navigation_speed_mps` mặc định 0,35,
được chọn trong 0,1..0,35 m/s cho lượt thử voice. Trên HTML bấm **Chuẩn bị
workflow voice MOVE → báo đã tới nơi** một lần. Workflow chuẩn
`WF_VOICE_ENDPOINT_V1` nhận endpoint/map/tốc độ qua tham số, không cố định a/b/c.

Trong cùng file AI private, có thể đặt `"navigation_ready_timeout_sec": 6`;
mặc định 6 giây, cho phép 0,5..15 giây. Khi xác nhận điểm đến mà phiên map cũ
tạm mất READY, voice chờ trong giới hạn này để phiên sẵn sàng trở lại; không
tạo phiên map mới hoặc khởi tạo lại AMCL. Phiên đóng/hết hạn, map hay revision
khởi tạo thay đổi vẫn bị từ chối;
MOVE tiếp tục kiểm tra pose và cảm biến trực tiếp trước khi chạy bánh.

Mở map đúng nơi robot đứng, tạo phiên tạm và chạy navigation với session ID
HTML cung cấp. Chờ READY rồi mở terminal khác, chạy `voice_audio.py chat`
như trên. Câu Việt/Anh hoặc trộn ngôn ngữ như “dẫn tôi tới point b”, “take me
to point b” được đối chiếu với endpoint của map hiện tại. Danh sách lấy qua
`GET /api/v1/pi/voice/navigation-destinations` bằng token DEVICE; không cần
gắn room, bật publicVisible hoặc thêm map vào RAG. Tên nghe chưa rõ do AI
đối chiếu danh mục thật; Pi/BE vẫn kiểm tra ID/map khi xác nhận.

Robot đọc lại tên điểm. Nói “xác nhận”/“confirm”/“yes” trong 60 giây để gửi
`POST /api/v1/pi/voice/navigation-requests`; nói “hủy”/“cancel” để bỏ. BE tạo
request/job NAVIGATION có MOVE → INTERACT/SPEAK. Chỉ MOVE thành công mới đọc
“đã tới điểm”; MOVE thất bại dừng tuyến, không phát câu thành công. Chat kết
thúc sau khi gửi yêu cầu, nhường loa cho job. Chạy lại chat để gửi lượt tiếp
theo trong cùng phiên map, không khởi tạo lại AMCL. Ctrl+C terminal chat
không hủy job đã gửi; hủy job trên HTML hoặc API cancel nếu cần dừng robot.

STT của chat dùng `languageCode=auto` để nhận Việt/Anh; các job thu âm cũ
vẫn giữ `vi`/`en`. BE tạo workflow từ mẫu chuẩn, không nhận graph do LLM
viết. Mỗi lượt có Idempotency-Key; phản hồi tạo request không rõ ràng giữ
`voice-action-pending.json` để nhân viên đối chiếu, không tự gửi job mới.
LLM không được mở servo hoặc gửi vận tốc.

## Voice thường trực trong phiên map

`voice_chat.enabled=true` trong worker config bật một tiến trình voice phụ do
supervisor quản lý khi chạy navigation. Không tạo thêm ROS node hoặc MQTT worker.
Ví dụ cấu hình (file AI vẫn giữ riêng bên ngoài repo):

```json
"voice_chat": {
  "enabled": true,
  "config_file": "/home/ubuntu/mar-secrets/voice-ai.json",
  "idle_settle_sec": 2,
  "start_timeout_sec": 12,
  "max_turns": 10
}
```

Mở phiên map trên HTML rồi chạy đúng lệnh `navigation_session.zsh` của phiên.
Có thể dùng `--voice-chat` hoặc `--no-voice-chat` để đổi riêng lượt chạy này.
Không mở thêm `voice_audio.py chat` khi listener đã chạy; xem
`pi/state/stack/voice.log`, trạng thái `VOICE_STATE WAIT_WAKE` là đang chờ gọi.
Từ khóa hiện tại là **“M A”**, đọc “Mờ A” hoặc “Em A”. Đặt cấu hình trong
`/home/ubuntu/mar-secrets/voice-ai.json` và dùng cùng `AI_WAKE_PHRASE` trên server AI:

```json
"wake": {
  "mode": "local_ai",
  "phrase": "M A",
  "vad_mode": 2,
  "vad_threshold": 0.004,
  "noise_margin_db": 6,
  "silence_sec": 0.65,
  "max_seconds": 4
}
```

Pi dùng WebRTC VAD trong `pi/requirements.txt` để loại tiếng không phải lời nói,
ước lượng tiếng nền và ưu tiên lời nói nổi bật hơn nền (mặc định biên độ gấp
khoảng 2 lần, tương ứng 6 dB). Có bộ đệm 300 ms trước câu gọi và chờ ngắt câu
650 ms để giữ hai chữ cái trong cùng đoạn. Đây là phân đoạn tiếng nói, không
tách riêng được người nói lớn nhất khi nhiều người nói chồng nhau, và không
xóa mọi tiếng ồn trong đoạn gửi đi. Không tự tăng gain micro.

Server AI cần `AI_WAKE_ENABLED=true`, `AI_WAKE_PHRASE=M A`,
`AI_WAKE_ALIASES=em a,mờ a` và `AI_WAKE_IGNORE_TONE=true`. Pi chỉ gửi đoạn có
tiếng nói tối đa 4 giây qua HTTPS, xử lý trong RAM. Log `VOICE_WAKE_AUDIO` chỉ ghi
mức âm nền, ngưỡng, độ dài và dấu hiệu vỡ tiếng; không ghi audio hoặc lời người
dùng. Cấu hình cũ không có `wake.mode` vẫn nhận sự kiện từ firmware WonderEcho
qua `/dev/wonderecho`.

Voice chỉ nhận khi robot IDLE. Khi có tín hiệu đánh thức, BE kiểm tra còn job
đang chạy hay cần recovery không, kể cả bước chờ do server xử lý. Admin vẫn dùng
workflow/API hiện có trên cùng map. Khi Pi nhận step mới, voice nhường micro/loa;
không đổi trạng thái job hoặc hủy MOVE vì kết thúc hội thoại. Sau job, listener
tự trở lại. Nói “tạm biệt”, “không cần nữa”, “goodbye” hoặc im lặng 12 giây để
đóng hội thoại. Phiên map/AMCL được giữ nguyên. Dẫn đường cần xác nhận và vẫn dùng
workflow MOVE → SPEAK đã publish.

Trạng thái worker trao đổi trong `/dev/shm` (RAM), không đọc DB trong vòng chờ.
Không ghi transcript vào log voice của stack, không giữ file thu âm của người
dùng. Lịch sử văn bản tiếp tục về BE. Lỗi AI/micro không dừng stack di chuyển.
Chưa có phiên map thì chức năng thường trực này không tự bật; CLI thu/phát/chat
thủ công vẫn dùng được. `Ctrl+C` stack dừng cả tiến trình voice và thiết bị thu/phát.

## Điện áp và phần trăm pin

RRC Lite phát `/ros_robot_controller/battery` dạng `UInt16`, đơn vị mV.
Worker gửi số đo trong `HEARTBEAT.payload.telemetry.battery`; không thêm MQTT
client, ROS node hoặc file lịch sử pin. `value` và `age_sec` giữ tương thích;
các trường mới gồm `unit`, `voltage_v`, `percent`, `estimate_method`,
`estimate_status`, `profile_id`, `curve_calibrated` và thông số cụm pin.
Pi chỉ gửi phần mở rộng sau khi `HELLO_ACK` từ BE có
`batteryTelemetryVersion=1`; với BE cũ, heartbeat/ROBOT_HEALTH giữ nguyên dạng
raw và `% = null` để không bị từ chối vì trường dữ liệu chưa hỗ trợ.

Phần `battery` trong hardware profile đang dùng cấu hình số cell nối tiếp/song
song, dung lượng mỗi cell và bảng `cell_voltage_curve` theo điện áp **mỗi cell**.
Ví dụ trong `pi/config/site.example.json` dành cho 4 cell 5330 mAh mắc 2S2P:
7,4 V danh định, 10660 mAh, đầy 8,4 V. Chỉ đặt `enabled=true` khi đúng cụm pin;
cấu hình cũ không có phần này tiếp tục gửi số đo raw, chưa ước tính %.

Bộ lọc median 5 mẫu và EMA 15 giây giảm dao động khi motor tải; cần 3 mẫu hợp lệ
trước khi có %. Mẫu quá 60 giây hoặc ngoài dải điện áp sẽ làm % về
`null` và khởi tạo lại bộ lọc khi nhận mẫu mới. Đọc API/heartbeat không gia hạn
thời điểm đo. Tính năng này chỉ cung cấp telemetry, không đổi chính sách MOVE,
điều khiển motor hay ngưỡng bảo vệ pin/BMS.

`curve_calibrated=false` là bảng Li-ion 4,2 V ban đầu, chưa phải phép đo dung
lượng hay đường cong đã đo riêng cho Samsung 58E. Dung lượng trên nhãn không
xác định được chính xác % từ điện áp dưới tải. Đối chiếu với đồng hồ đo trước
khi chỉnh `voltage_gain`/`voltage_offset_v`; chỉ bật `curve_calibrated` sau khi
đã đo và thay bảng SOC cho chính cụm pin. Không suy ra dòng sạc/xả hoặc thời
gian còn lại từ số đo này.

Sau khi deploy BE tương ứng, ADMIN/STAFF đọc
`GET /api/v1/robots/{robotId}/battery` với Bearer token của người dùng. Response
có điện áp, %, độ mới và phương pháp ước tính; dữ liệu cũ không được trình bày
như số đo hiện tại. Pi tiếp tục dùng device token riêng và MQTT đã cấu hình.

## Âm lượng và kết nối lại

`speech.default_volume` trong worker config nhận số nguyên 0–100, mặc định 100. Hội thoại AI dùng cùng mức này cho stream PCM, WAV có sẵn và fallback; hướng dẫn QR/face cũng dùng mặc định này. `staff_qr_guidance.volume` và `data.volume` của step vẫn có thể chọn mức riêng, kể cả 0 để tắt tiếng. ALSA Speaker là tầng âm lượng riêng, không thay bằng việc tăng microphone gain. Không bật lại đọc path trong MOVE.

Ngày 23/11/2026 đã cài 6 file production âm thanh/worker lên Pi 192.168.1.154, đặt default/QR 100 và xác nhận ALSA Speaker 100%, không mute; giữ stack dừng, không cài file test. Bản trước nằm tại `pi/state/backups/audio-queue-20261123T132049Z/`. Đã giữ các nhánh hỏi ca trực đang có trên Pi khi đồng bộ source voice về laptop.

HELLO hợp lệ đặt lại thời gian chờ reconnect về 1 giây; các lần HELLO/kết nối lỗi liên tiếp vẫn tăng dần tới 30 giây. Pi lưu kết quả từng step và hủy operation; BE giữ hàng đợi job. Lỗi MOVE đã xác nhận dừng không tự đóng phiên navigation dùng chung. `RECOVERY_REQUIRED` hoặc motion chưa xác minh vẫn chặn nhận operation tiếp theo.

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
