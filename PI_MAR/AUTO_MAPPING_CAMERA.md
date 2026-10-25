# Tự quét map, tránh vật cản và camera — 23/10/2026

## Kết quả và phạm vi

Đã bổ sung Pi + BE + trang `http://127.0.0.1:8080/mapping.html`:

1. Tạo **một job quét map** như trước; Pi START tạo phiên SLAM mới.
2. Khi job đang WAIT `MAPPING_SAVE_REQUESTED`, bấm **Bắt đầu tự quét**.
3. Pi đọc LiDAR và TF, tự đi tới/chọn hướng. Vòng điều khiển 20 Hz chạy tại Pi.
4. Trang gia hạn quyền chạy mỗi khoảng 300 ms. Quyền có hạn tối đa 900 ms trên Pi, bị giới hạn thêm bởi thời hạn gói MQTT. Không phát lại lệnh cũ, không tự chạy lại sau lỗi.
5. Dừng khi hết thời gian, khi đi qua gần điểm đầu nếu được chọn, hoặc khi dữ liệu/phiên/đường đi không còn hợp lệ.
6. Nếu tích **Lưu map và kết thúc...**, chỉ hai lý do dừng bình thường `TIME_LIMIT`/`RETURNED_TO_START` mới gửi yêu cầu lưu. Các lỗi cảm biến hoặc mất kết nối không tự coi là hoàn thành.
7. Luồng cũ tiếp tục: **SAVE trên Pi → upload HTTPS → COMMIT_MAP trên BE → STOP phiên → Job SUCCEEDED → Xem map đã lưu / PNG**.
8. Phần endpoint, xem vị trí và workflow MOVE đã có vẫn ở mục bản đồ đã lưu.

Không tạo StepType mới. Điều khiển tự quét dùng lane `MAPPING_CONTROL` có trường `autonomy`, trong chính job mapping hiện tại. SAVE/COMMIT/STOP vẫn dùng các step của workflow đang có. Chạy module LiDAR/joystick hãng song song, cùng ghi `cmd_vel`, sẽ gây tranh quyền; bản này chỉ có một bộ phát vận tốc qua gate hiện có.

**Đây là tự khám phá theo khoảng hở LiDAR có giới hạn, chưa phải thuật toán lập kế hoạch quét phủ toàn phòng.** Không có lời khẳng định đã quét hết; `coverage_complete` luôn false. Khi bị kẹt hoặc không có cung rẽ hợp lệ, Pi dừng và báo nguyên nhân.

## Xe kéo của bạn

Thông tin bạn cung cấp: thân 212 × 171 mm; thùng khoảng 150 × 150 mm; đoạn nối 20–30 mm; thùng có khớp, xoay riêng. Chiều cao thân/thùng không được dùng như tọa độ Z để tính map LiDAR 2D.

Chế độ `articulated`:

- Chỉ đi tới và rẽ theo cung có bán kính tối thiểu cấu hình; **không quay tại chỗ, không tự lùi**.
- Kiểm tra vùng bao cả thân và xe kéo tại mọi góc khớp, cùng các điểm trên cung đi tới. Vùng tròn bảo thủ có thể khiến robot dừng sớm trong chỗ hẹp.
- LiDAR bị che hoặc nhìn thấy chính thùng trong vùng nguy hiểm sẽ làm dừng; không xóa các tia này để giả định rằng đường trống.
- Chưa có cảm biến góc khớp: code không đo được góc gập, độ trượt bánh hay tự chứng minh cơ cấu không gập quá mức. Phải kiểm tra bán kính rẽ với cụm thật. Bản này chưa cung cấp bộ lập kế hoạch động học xe kéo cho Nav2.

Nếu `base_footprint` nằm đúng tâm thân, đầu nối nằm sau thân và thùng thật gần kích thước trên, bộ số **dự thảo** để xem kế hoạch là:

```
front = 0.116 m; rear = 0.320 m; left = right = 0.180 m
```

Đây là vùng bao bảo thủ cho thùng xoay, không phải bốn nửa kích thước thùng khi đang thẳng. Phải kiểm tra lại vị trí `base_footprint`, mép ngoài, phụ kiện và tầm quét thực. `turn-radius=0.60` trong ví dụ dưới là **giá trị thử để xem plan, chưa phải bán kính đã đo trên robot của bạn**.

## 1. Upload và vá code Pi

Nếu đang có job cần phục hồi, xử lý theo mục cuối trước khi bắt đầu một demo mới. Dừng terminal `robot.zsh run` bằng Ctrl+C, chờ toàn bộ STOP hoàn tất; không rút điện giữa lúc lưu.

### Windows PowerShell

```powershell
$piIp = '10.3.66.91'  # thay bằng IP SSH hiện tại nếu đã đổi mạng
scp 'C:\Users\LENOVO\Downloads\MAR_AutoMapping_Camera_PI_2026_10_23.zip' "pi@${piIp}:/tmp/"
ssh "pi@$piIp"
```

### Pi HOST — prompt pi@raspberrypi, chưa vào container

```sh
sudo docker start MentorPi
sudo docker cp /tmp/MAR_AutoMapping_Camera_PI_2026_10_23.zip MentorPi:/tmp/
sudo docker exec -it -u ubuntu \
  -w /home/ubuntu/mediassist-mqtt-2026-10-08 \
  MentorPi zsh
```

### Bên trong container MentorPi

Giữ nguyên từng dòng, đặc biệt đoạn Python nhiều dòng:

```sh
cd /home/ubuntu/mediassist-mqtt-2026-10-08
zsh tools/robot.zsh stop
python3 -m zipfile -e /tmp/MAR_AutoMapping_Camera_PI_2026_10_23.zip /tmp/mar-auto-camera-20261023
python3 /tmp/mar-auto-camera-20261023/install_pi_patch.py
python3 /tmp/mar-auto-camera-20261023/install_pi_patch.py --apply
```

Installer kiểm tra SHA-256, từ chối nếu stack còn chạy, sao lưu code từng file, kiểm tra cấu hình với Python venv sau khi vá và khôi phục code nếu bước kiểm tra thất bại. Installer **không chép đè config đang dùng, token, CA, device secret, journal hoặc map**. Bản sao code ở `pi/state/code-backups/<timestamp>/`.

Không cần tạo device token theo IP. Nếu đổi IP BE, dùng công cụ network cũ để cập nhật HTTPS/upload/TTS; chứng chỉ CA là file công khai của CA cấp chứng chỉ HTTPS, không lấy từ API token. DNS lỗi `gaierror` cần sửa DNS container như trước, không giải quyết bằng đổi token.

## 2. Cấu hình camera khi stack đã dừng

Source Hiwonder gửi kèm dùng `ascamera`: RGB/depth **640×480, fps=15**. 20 FPS là một mức thử đã có trong công cụ, chưa được chứng minh trên camera của bạn. Không suy ra FPS thực từ giá trị cấu hình.

```sh
cd /home/ubuntu/mediassist-mqtt-2026-10-08
pi/.venv/bin/python tools/configure_camera.py --fps 15 --width 640 --quality 60
pi/.venv/bin/python tools/configure_camera.py --fps 15 --width 640 --quality 60 --apply
```

Lệnh đầu chỉ kiểm tra; lệnh sau sao lưu và cập nhật đúng launch camera đang dùng. Nếu báo source ROS shell/ascamera chưa đúng, mở lại `MentorPi zsh` để nạp môi trường; không tự tìm một launch khác rồi sửa nhầm. Độ sâu, TF và các tính năng khác được giữ nguyên.

Bản vá camera:

- Nhận ROS trên executor riêng; bộ nén JPEG không chặn vòng nhận ảnh.
- QoS BEST_EFFORT, depth=1; một mẫu ảnh mới nhất, không xếp hàng ảnh cũ.
- Kiểm tra timestamp camera, bỏ ảnh nguồn đã trễ; stream giữ giới hạn tuổi ảnh 350 ms **trước khi gửi**, không phải cam kết trễ tới màn hình dưới 350 ms.
- Nén một lần và chia cho các viewer; ngắt viewer ghi quá chậm; giới hạn bộ đệm TCP.
- Ảnh MQTT vẫn là thumbnail 0,5 Hz mặc định. Không dùng nó để đánh giá độ mượt video LAN.

Nếu Wi-Fi yếu/CPU cao, giảm video xuống 320×240 nhưng giữ 15 FPS:

```sh
pi/.venv/bin/python tools/configure_camera.py --fps 15 --width 320 --quality 50 --apply
```

Chỉ thử 20 FPS sau khi mức 15 đã chạy ổn và kiểm tra số đo:

```sh
pi/.venv/bin/python tools/configure_camera.py --fps 20 --width 320 --quality 50 --apply
```

Mỗi lần thay cấu hình phải dừng stack rồi khởi động lại một lần. Không cần tạo container/bản project mới.

## 3. Xem plan và bật tự quét

Ví dụ này **chỉ in kế hoạch**, chưa bật tự chạy:

```sh
pi/.venv/bin/python tools/configure_autonomous_mapping.py \
  --enable --vehicle articulated \
  --front 0.116 --rear 0.320 --left 0.180 --right 0.180 \
  --turn-radius 0.60 --speed 0.08 --angular 0.20 --duration 180
```

Sau khi thay bằng số đo đã kiểm tra, chạy lại cùng lệnh và thêm `--apply` để ghi cấu hình. Công cụ không nâng các cờ commissioning, không tự đổi US100 required/optional, không nâng trần tốc độ phần cứng. Nếu profile đang giới hạn dưới mức yêu cầu, giảm `--speed`/`--angular`; không sửa cờ kiểm chứng để ép chạy.

Tắt riêng tự quét, giữ tay cầm/web và map đã lưu:

```sh
pi/.venv/bin/python tools/configure_autonomous_mapping.py --disable --apply
```

Tốc độ tự chạy tối đa trong code 0,15 m/s; xe kéo rẽ theo cung dùng tối đa 0,06 m/s và còn bị giới hạn bởi bán kính rẽ/profile. Chạy với xe kéo chậm hơn robot trần là có chủ ý. SSH không đặt tốc độ; `robot.zsh run` chuyển sang bộ điều khiển/profile của đồ án.

Nếu muốn chỉnh tay cầm trong giới hạn hiện có, khi stack đã dừng:

```sh
pi/.venv/bin/python tools/configure_gamepad.py --source gamepad --forward 0.15 --reverse 0.10 --angular 0.40
```

Đây là cấu hình cho điều khiển tay, **không phải thông số rẽ an toàn đã xác nhận cho xe kéo**. Giới hạn chạy liên tục 3 giây và yêu cầu trả cần về giữa vẫn giữ để xử lý tay cầm RF có thể lặp trạng thái cuối. Không bật thêm joystick hãng ghi trực tiếp chassis song song.

## 4. Khởi động và kiểm tra

```sh
zsh tools/robot.zsh run
```

Giữ terminal này mở. Terminal container thứ hai:

```sh
cd /home/ubuntu/mediassist-mqtt-2026-10-08
tail -n 45 pi/state/stack/worker.log
timeout 8s ros2 topic hz /scan_raw --window 10
timeout 8s ros2 topic hz /map --window 10
pi/.venv/bin/python tools/check_camera_stream.py --seconds 10
```

Camera checker mở một viewer tạm, chỉ đọc video và `/health`, không gửi lệnh robot. Trong trình duyệt laptop chọn **Xem video trực tiếp** với:

```
http://IP_PI:8081/stream?topic=/ascamera/camera_publisher/rgb0/image
http://IP_PI:8081/health
```

Đọc các trường `/health`:

| Trường | Cách hiểu |
|---|---|
| `targetFps` | FPS yêu cầu, không phải FPS đạt được |
| `sourceReceivedFps` | Tốc độ ảnh hợp lệ tới subscriber khi có viewer |
| `encodedFps` | Tốc độ ảnh nén thực |
| `lastEncodeMs` | Thời gian nén ảnh gần nhất |
| `lastReceiveToEncodeMs` | Thời gian từ callback nhận tới nén xong |
| `lastClientWriteMs` | Thời gian ghi gần nhất cho một viewer; không phải thời gian hiển thị |
| `lastSourceError` | Ví dụ `CAMERA_SOURCE_STALE`: ảnh nguồn đã trễ/sai giờ |

Nguồn thấp → kiểm tra driver/USB/tải Pi. Nguồn đủ nhưng encode thấp → giảm độ phân giải/quality. Cả hai đủ mà laptop vẫn trễ → kiểm tra Wi-Fi, viewer và trình duyệt. Dùng một tab video khi đo. Kiểm tra số đo trước/sau trong cùng điều kiện; chưa có đo camera thật trong lần sửa này vì SSH Pi đang timeout.

## 5. Các field trên trang và Swagger

- Robot ID: `0e3f1083-2546-4f45-8004-5eda5cb75a6f`.
- Bấm **Chuẩn bị workflow quét và lưu** để lấy workflow đang publish; workflow trước đây: `74ecb162-b119-3546-2ac6-a28109fe7518`. Không sửa workflow đã publish nếu đang có job dùng nó.
- Floor ID dùng đúng tầng của lần quét; tầng trước đây: `c214abc7-c887-11f1-904a-f63b6c16c38d`.
- Map name đặt tên mới dễ phân biệt; Job ID và mappingSessionId lấy từ kết quả tạo/theo dõi, không dùng lại ID phiên cũ.
- Token dùng ADMIN hiện còn hạn; token không được ghi vào bản vá.
- Tự quét: 180 giây mặc định; tùy chọn dừng gần điểm đầu, tùy chọn lưu sau dừng bình thường. Nút chỉ bật khi Pi báo đã cấu hình và heartbeat còn mới.

API mới:

```
POST /api/v1/jobs/{jobId}/mapping-autonomy
```

```json
{
  "mappingSessionId": "UUID_PHIEN_HIEN_TAI",
  "autonomy": {
    "action": "START",
    "runId": "UUID_MOI_CHO_LAN_TU_CHAY",
    "maxDurationSec": 180,
    "stopOnReturn": false
  },
  "controlSessionId": "UUID_CUA_TRANG_DIEU_KHIEN",
  "controlSequence": 1,
  "requestedAt": "THOI_GIAN_UTC_HIEN_TAI"
}
```

`START`, `RENEW`, `STOP` giữ cùng runId/options; sequence tăng mỗi request. Swagger bấm một lần **không thể duy trì chạy 180 giây**: Pi dừng khi hết lease. Dùng trang HTML để gia hạn tự động. HTTP 202 chỉ là BE nhận/gửi lệnh; xem `autonomy` từ `GET /api/v1/jobs/{jobId}/mapping-session?includePreview=false` để biết Pi thực hiện hay đã dừng.

## 6. Flag dừng và màu bản đồ

| Flag/lý do | Ý nghĩa |
|---|---|
| `TIME_LIMIT` | Đã hết thời gian tự chạy trên Pi |
| `RETURNED_TO_START` | Đã rời xa, đi đủ quãng đường/thời gian rồi đi qua gần pose lúc START tự chạy và gần đúng hướng ban đầu |
| `CONTROL_LEASE_EXPIRED` | Mất quyền chạy/gia hạn; không tự tiếp tục |
| `LIDAR_STALE`, `MAPPING_MAP_STALE`, `MAPPING_POSE_NOT_READY` | Thiếu dữ liệu mới để tiếp tục |
| `TRAILER_NO_CLEAR_FORWARD_ARC`, `TRAILER_FORWARD_ARC_BLOCKED` | Không có cung đi tới hợp lệ cho vùng bao xe kéo |
| `ROTATION_ENVELOPE_BLOCKED`, `TURN_TIMEOUT` | Không đủ khoảng hở hoặc không thoát được hướng đang bị chắn |
| `GAMEPAD_OVERRIDE` | Chạm cần khi đang tự chạy: dừng chế độ tự chạy; bấm Dừng tự chạy và trả cần về giữa để điều khiển tay |
| Job `SUCCEEDED` + COMMIT có map_id | Luồng lưu/đăng ký/đóng phiên đã hoàn tất; có thể xem PNG |

`RETURNED_TO_START` là điều kiện nhận biết lúc đi qua điểm đầu, **không phải robot tự lập đường về**. Pose điểm đầu lấy trong phiên SLAM mới, không dùng tọa độ endpoint của map cũ khi chưa có phép căn chỉnh hai hệ tọa độ.

Trắng: vùng đã quan sát là trống. Đen: vật cản/tường. Xám: chưa biết. Phần ngoài phòng kín có thể mãi màu xám vì LiDAR không xuyên tường; không lấy tỷ lệ trắng trên toàn hình chữ nhật làm điều kiện đã quét xong.

## 7. Job đang cần khôi phục và triển khai BE

Lúc kiểm tra, job `28a0abb2-8df8-4a35-bd62-45d298ed86dd` đang `RECOVERY_REQUIRED`, phiên `009ee186-b916-4ddd-84dc-3ed642a99006` còn báo mở; robot OFFLINE. Bản vá không tự xóa journal, đóng cưỡng bức job hoặc bịa xác nhận dừng. Khi Pi lên lại, kiểm tra bằng chứng recovery và dùng nút **Đóng job lỗi đã có bằng chứng dừng**. Chỉ tạo job mới khi job cũ đã terminal, phiên đóng và robot dispatchReady.

BE source đang dùng: `C:\Users\LENOVO\Downloads\MAR_EMQX_Demo\BE_MAR`; bản Git: `C:\Users\LENOVO\OneDrive\Tài liệu\GitHub\BE_MAR`.

Code BE/HTML đã cập nhật và kiểm thử. Xem `verification.json` để biết trạng thái triển khai cuối cùng. Script `check-be-restart.py` chỉ đọc trạng thái: yêu cầu không có QUEUED/ACCEPTED/RUNNING và Pi MQTT offline; recovery được giữ nguyên qua restart.

Nếu cần tự build/restart khi đã dừng robot:

```powershell
Set-Location 'C:\Users\LENOVO\Downloads\MAR_EMQX_Demo\BE_MAR'
docker compose -p mar-emqx-demo --env-file .env build backend
docker compose -p mar-emqx-demo --env-file .env up -d --no-deps backend
# Đợi /actuator/health trả UP rồi reload upstream nginx:
docker compose -p mar-emqx-demo --env-file .env exec -T reverse-proxy nginx -s reload
Start-Process 'http://127.0.0.1:8080/mapping.html'
```

Không dùng `down -v`. Không đổi CA/token chỉ để cập nhật code.

## 8. Khôi phục code nếu cần

Dừng stack. Installer đã in đường dẫn backup. Trước khi rollback code, nếu từng bật tự quét, tắt bằng tool mới:

```sh
pi/.venv/bin/python tools/configure_autonomous_mapping.py --disable --apply
python3 /tmp/mar-auto-camera-20261023/install_pi_patch.py \
  --rollback /home/ubuntu/mediassist-mqtt-2026-10-08/pi/state/code-backups/TIMESTAMP_DA_IN
```

Rollback từ chối nếu code đã thay đổi sau lúc vá, để không ghi đè sửa mới của bạn. Config camera có backup riêng trong `pi/state/camera-backups`; code rollback không tự thay cấu hình/launch hãng đã chỉnh bởi `configure_camera.py`. Sao lưu config hiện hành vẫn ở cạnh file `.bak` khi công cụ có ghi thay đổi.

## Kiểm thử

Chi tiết và số lượng cuối cùng ở `verification.json`; log trong cùng folder. Có kiểm thử API/cơ sở dữ liệu MySQL tách biệt, hợp đồng MQTT hai đầu, điều phối một nguồn vận tốc, lỗi lease/scan/map/pose, xe kéo chỉ rẽ cung, mã hóa JPEG thật, HTTP MJPEG và trang HTML giả lập. **Không có kiểm thử chuyển động hay đo FPS trên Pi thật trong lần này.**
