# Bản vá tự quét và dẫn đường theo phiên — 24/10/2026

## Cần vá những đâu?

**Cả Pi + BE + HTML.** Pi thực thi SLAM/Nav2/AMCL, BE quản lý phiên map, xác nhận profile và điều phối workflow; HTML chọn map/điểm khởi tạo/endpoint. Chỉ vá Pi sẽ thiếu API và hợp đồng dữ liệu mới. Source trên laptop đã cập nhật; Docker BE đang chạy chưa được rebuild. Pi đang tắt, chưa cài bản này và chưa kiểm thử chuyển động thật.

Source Pi: `C:\Users\LENOVO\OneDrive\Tài liệu\GitHub\RASPPI_MAR\PI_MAR`.

Source BE chạy demo: `C:\Users\LENOVO\Downloads\MAR_EMQX_Demo\BE_MAR`. Các file BE thay đổi cũng được đồng bộ sang Git `C:\Users\LENOVO\OneDrive\Tài liệu\GitHub\BE_MAR`. Không commit/push tự động.

## 1. Cách hiểu đúng về phiên và vị trí robot

- Thư viện BE giữ nhiều map và endpoint. Một robot dùng một map tại một thời điểm trong **phiên thử tạm**, mặc định 2 giờ, tối đa 8 giờ. Đổi địa điểm: đóng phiên, chọn map khác, mở phiên mới. Không ghi cứng map này vào cấu hình gốc Pi.
- Mỗi lần chạy navigation có thư mục riêng `pi/state/navigation-sessions/<sessionId>/<run>/`. Cấu hình MQTT/TLS/secret/hardware gốc được đọc để tạo bản tạm, không bị ghi đè. Journal thực thi vẫn dùng chung để không làm mất nhiệm vụ cần khôi phục.
- `Robot.currentMapId` trên BE phản ánh map thực tế Pi báo trong heartbeat. Tạo phiên không tự đánh dấu Pi đã nạp map. Quy tắc catalog hiện có vẫn là một map ACTIVE trên mỗi tầng; các map còn lại giữ trong thư viện, có thể kích hoạt lần lượt khi test.
- **Mở PNG/YAML không đủ biết robot đang ở đâu.** Không tự đặt robot tại (0,0). Sau khi nạp map, trạng thái là WAITING_FOR_PI/LOCALIZING; MOVE bị chặn cho đến READY.
- Cách khởi tạo nhanh: đặt robot tại vị trí biết trên sàn; trên HTML chọn chế độ khởi tạo, bấm/kéo trên map để nhập X/Y và hướng thân robot. Đây chỉ là ước lượng gửi `/initialpose` cho AMCL.
- Cách GLOBAL: phân bố hạt AMCL toàn map rồi đối chiếu LiDAR. Robot không tự quay để tìm vị trí trong giai đoạn khởi tạo này. Không gian đối xứng, hành lang giống nhau hoặc map ít đặc trưng có thể không hội tụ; dùng SEED để chỉ đúng khu vực. Không có thuật toán nào chứng minh tuyệt đối đúng phòng chỉ từ các hình học giống hệt nhau.
- Pi kiểm tra map/hash/file/geometry, vòng đời map_server và AMCL, TF/scan/odom mới, covariance, nhiều scan khớp tường map, tỷ lệ dữ liệu đã biết, thời gian ổn định; GLOBAL còn kiểm tra độ tập trung hạt. Vị trí hiển thị xanh chỉ xuất hiện sau các bằng chứng này. Mất bằng chứng thì ẩn pose và chặn/dừng MOVE.
- Endpoint thuộc hệ tọa độ map, có thể thêm trước hoặc sau khi định vị. Thêm/sửa endpoint không làm thay đổi vị trí robot. Pi nhận lại catalog có profile hash mới và ACK trước khi BE gửi MOVE.
- Lưu cả X/Y và yaw (radian); không cần Z cho dẫn đường trên sàn. Hướng camera phụ thuộc thêm góc lắp camera so với thân robot.
- Restart tiến trình navigation, hết hạn phiên hoặc đổi map cần phiên mới và xác minh lại vị trí; không phục hồi pose cũ một cách mặc định. Kết nối MQTT lại trong cùng tiến trình được phép nhưng phải đồng bộ profile lại.

## 2. Dữ liệu và luồng MOVE

1. ADMIN mở phiên bằng map ID. DEVICE chỉ tải bundle được cấp cho robot/phiên của nó, qua HTTPS với CA hiện có.
2. Pi kiểm tra SHA256 của YAML+PGM và metadata; khởi chạy driver hãng, EKF, map_server, AMCL, Nav2. Không chạy SLAM dựng mới trong phiên dùng map đã lưu.
3. HELLO/heartbeat mang `navigationSession`: session_id, map_id, map_asset_hash, expires_at. `telemetry.localization` mang ready, reason, session_id, initialization_revision, map/hash và age. `map_pose` chỉ fresh khi định vị hợp lệ.
4. BE gửi `BE-NAV-PROFILE-1`, gồm endpoint X/Y/yaw, map/hash/frame, navigation_session_id và initialization_revision. Pi kiểm tra map đang nạp thực tế rồi ACK profile hash.
5. HTML tạo navigation request với robotId + navigationSessionId + mapId + endpointId + workflowId. BE tạo job gắn đúng robot, không chọn robot khác nếu robot này bận.
6. Workflow vẫn dùng StepType **MOVE** cũ (`map_id`, `endpoint_id`, `mode`, `max_speed_mps`). Envelope thêm `navigationContext` đóng băng session/map/hash/revision/profile_hash. Hai phía chặn sai/đổi phiên, map, profile hay revision.
7. Nav2 lập đường trên costmap, DWB kiểm tra polygon thân và chọn vận tốc. Vận tốc qua đường riêng `/mediassist/nav2_raw` → velocity_smoother → `/mediassist/nav2_cmd_vel` → MAR chọn nguồn → gate/watchdog → controller hãng.
8. Thành công cần Nav2 SUCCEEDED và pose đã xác minh cách endpoint <= 8 cm, sai yaw <= 0,15 rad; dừng vật lý được xác nhận trước khi worker gửi kết quả. Kết quả có verified_pose và sai số. Mẫu workflow trên HTML vẫn giới hạn 0,05 m/s cho lần test đầu.

## 3. Tự quét và né vật cản

- Driver MS200, odom/EKF, SLAM và camera native `web_video_server` dùng nền hãng. Không bật thêm node hãng phát thẳng `/controller/cmd_vel` cạnh MAR.
- Phiên quét dùng SLAM sống và Nav2 planner/controller; không chạy AMCL/map_server tĩnh chồng lên. Explorer tìm frontier (ranh giới free/unknown), gửi điểm quan sát cho Nav2, lấy map mới rồi lập kế hoạch tiếp.
- Sửa lỗi đầu phiên bị chặn bởi vòng tròn bao lớn: kiểm tra polygon thân thật theo yaw để phân biệt đè vật cản thật với vùng đệm/cung quay. Nav2 kiểm tra toàn quỹ đạo; không dùng đường kết nối thô của explorer để lái xuyên khe hẹp.
- Chờ tối đa 8 giây nếu map chưa biết đủ quanh thân; vẫn đứng yên. Đè occupied thật, scan/TF cũ, mất lease, mất pose hoặc không có đường thì dừng và ghi lý do; không tô ô xám thành trắng để ép chạy.
- Giảm vận tốc tiến khi khoảng hở hành lang trước thân giảm; giới hạn gia tốc và DWB chọn đường/quay tránh. Hard-stop bổ sung là 10 cm tính từ LiDAR. Đây khác khoảng hở quanh thân và không đảm bảo phanh đủ ở mọi tốc độ; lần đầu thử 0,10 m/s, không xe kéo.
- Chỉ COVERAGE_COMPLETE khi không còn frontier đáng kể trong vùng free liên thông, map ổn định qua ít nhất 3 revision/15 giây và đủ thời gian/diện tích. Frontier không tới được, timeout, kẹt hoặc mất mạng không được coi là hoàn tất.
- Tường/vật cản đen và xám bên ngoài tường kín là bình thường. Không dùng điều kiện “hết màu đen”. Vùng bị đồ đạc che hoặc không tới được không thể bảo đảm đã khảo sát.
- Khi chọn tự lưu, Pi báo completed + bằng chứng dừng; BE phát sự kiện SAVE một lần và đi theo WAIT → SAVE_MAPPING → upload → COMMIT_MAPPING → STOP_MAPPING. Không báo đã lưu chỉ vì auto.finished=true. Browser vẫn phải mở để gia hạn lệnh tự chạy; đóng tab/mất mạng làm dừng. Việc xử lý SAVE sau khi BE đã nhận completion không phụ thuộc tab.
- Khi STOP lỗi, Pi lưu dữ liệu map/pose/scan view/config đồng bộ tại `<mapping.storage_dir>/diagnostics/<runId>.json` để kiểm tra, không gửi toàn bộ floorplan trong heartbeat.
- Bản cũ xuất pixel205 unknown nhưng free_thresh=.25 khiến Nav2 đọc thành free. Bản mới dùng .196. Với map cũ, nút “Tạo bản map sửa ô xám cho Nav2” tạo map/hash/endpoint ID mới, giữ bản gốc nguyên vẹn. Kích hoạt và dùng bản mới; không tái dùng endpoint ID của bản gốc.

## 4. Cài code BE và khởi động lại

Đóng/hủy job hiện tại đúng luồng trước khi restart. Không xóa database/journal để bỏ trạng thái.

Trên laptop, source demo đã có bản sửa; chạy PowerShell:

```powershell
Set-Location -LiteralPath 'C:\Users\LENOVO\Downloads\MAR_EMQX_Demo\BE_MAR'
docker info --format '{{.ServerVersion}}'
docker compose -p mar-emqx-demo build backend
if ($LASTEXITCODE -ne 0) { throw 'Build BE failed; keep the existing container.' }
docker compose -p mar-emqx-demo up -d --no-deps backend
if ($LASTEXITCODE -ne 0) { throw 'BE startup failed; inspect backend logs.' }
docker compose -p mar-emqx-demo logs --tail 100 backend
```

Mở trang sau khi health UP:

```powershell
Invoke-RestMethod 'http://127.0.0.1:8080/actuator/health'
docker compose -p mar-emqx-demo exec -T reverse-proxy nginx -t
if ($LASTEXITCODE -ne 0) { throw 'Nginx configuration check failed.' }
docker compose -p mar-emqx-demo exec -T reverse-proxy nginx -s reload
Start-Process 'http://127.0.0.1:8080/mapping.html#mapNavigation'
```

Chỉ reload reverse-proxy khi health BE đã UP (nếu BE đang khởi động thì chờ rồi kiểm tra lại). Reload giúp nginx nhận địa chỉ container backend sau khi recreate. Các lệnh này dùng reverse-proxy đang có trong demo HTTPS.

Nếu dùng source BE ở máy/folder khác, giải nén ZIP BE và chạy `Install-BePatch.ps1 -Root '<folder BE>'` để kiểm tra trước, rồi thêm `-Apply`. Installer kiểm tra hash, sao lưu từng file, không đụng `.env`, TLS, Docker volume hay database. Flyway tự thêm V37 lúc BE mới khởi động; V35/V36 giữ nội dung đã tồn tại trong Git trước bản vá. Không chạy `docker compose down -v`.

Giữ HTTPS và CA hiện có nếu IP BE không đổi. Nếu đổi mạng, dùng bộ Update-MarNetwork trước đây để cập nhật origin và SAN; token DEVICE gắn robot, không gắn IP. Bản vá này không thay DNS, mật khẩu hay CA.

## 5. Upload và cài Pi khi bật lại

IP Pi gần nhất: 192.168.1.22. Nếu DHCP đổi IP sau khi bật lại, thay biến `$piIp` bằng IP mới đã xác nhận.

PowerShell laptop:

```powershell
$piIp = '192.168.1.22'
scp 'C:\Users\LENOVO\Downloads\MAR_NavigationSession_PI_2026_10_24.zip' "pi@${piIp}:/home/pi/"
ssh "pi@$piIp"
```

Trong **HOST Pi**, ngoài container:

```sh
sudo docker start MentorPi
sudo docker cp /home/pi/MAR_NavigationSession_PI_2026_10_24.zip MentorPi:/tmp/
sudo docker exec -it -u ubuntu -w /home/ubuntu/mediassist-mqtt-2026-10-08 MentorPi zsh
```

Trong **container MentorPi**: dừng terminal `robot.zsh run` cũ bằng Ctrl+C, chờ STOP đầy đủ; không mở worker thứ hai. Sau đó:

```sh
cd /home/ubuntu/mediassist-mqtt-2026-10-08
python3 -m zipfile -e /tmp/MAR_NavigationSession_PI_2026_10_24.zip /tmp/mar-navigation-session-20261024
pi/.venv/bin/python /tmp/mar-navigation-session-20261024/install_pi_patch.py
pi/.venv/bin/python /tmp/mar-navigation-session-20261024/install_pi_patch.py --apply
```

Nếu CODE_BASELINE_MISMATCH: dừng tại đó và đối chiếu đúng file báo; không ép ghi đè file đã sửa riêng. Installer nhận các bản vá đã biết, không cần cài chồng từng ZIP cũ. Gói chỉ chứa source/contracts; không chứa cấu hình site, khóa/token, map hay dữ liệu test thật.

Kiểm tra package ROS đã có từ hãng:

```sh
ros2 pkg prefix nav2_controller
ros2 pkg prefix nav2_planner
ros2 pkg prefix nav2_bt_navigator
ros2 pkg prefix nav2_velocity_smoother
ros2 pkg prefix nav2_amcl
ros2 pkg prefix nav2_map_server
ls -l /home/ubuntu/ros2_ws/src/navigation/config/nav2_params.yaml
ls -l /home/ubuntu/ros2_ws/src/navigation/config/nav2_controller_dwb.yaml
```

Các bước này chỉ kiểm tra; không cài/chạy thêm stack hãng. Nếu thiếu package/YAML, xử lý phụ thuộc trước khi run.

### Quét map mới

Không xe kéo như đã thống nhất; preset kích thước bao thân trước/sau .12 m, trái/phải .10 m. Khi stack đã dừng, xem kế hoạch rồi áp dụng:

```sh
pi/.venv/bin/python tools/configure_bare_mecanum.py --trailer-removed --source gamepad --auto-speed 0.10
pi/.venv/bin/python tools/configure_bare_mecanum.py --trailer-removed --source gamepad --auto-speed 0.10 --apply
zsh tools/robot.zsh run
```

Preset giữ chứng nhận commissioning đã có, không tự biến các cờ kiểm chứng phần cứng thành true. US100 tiếp tục chẩn đoán tùy chọn; LiDAR/odom/gate/watchdog vẫn kiểm tra. Camera native đã chọn giữ nguyên. Muốn dùng nút tiến/lùi web thay gamepad, chọn `--source mqtt` khi dừng stack, không bật hai nguồn song song.

Trên HTML: đăng nhập ADMIN → chọn đúng robot/floor → tạo job quét → chờ MAPPING → bật tự quét và tự lưu. Để cần giữa; robot dừng khi gamepad override hoặc lease/scan/pose mất. Kết thúc thành công chỉ khi map đã COMMIT; dùng nút xem map đã lưu.

### Nạp map đã lưu và MOVE tới endpoint

1. Kết thúc/hủy job quét đúng luồng, chờ không còn job RUNNING/RECOVERY_REQUIRED. Ctrl+C terminal run và chờ STOP.
2. HTML phần Endpoint: nhập map ID, tải map/endpoint; tạo bản sửa unknown nếu cần, kích hoạt map mới.
3. Bấm “Mở phiên dùng map tạm cho robot”. Sao chép **nguyên lệnh do HTML hiện ra**, có session UUID thật. Dạng lệnh:

```sh
cd /home/ubuntu/mediassist-mqtt-2026-10-08
zsh tools/navigation_session.zsh --session-id <UUID_HIEN_TREN_HTML>
```

Không dán nguyên dấu `<...>`; copy lệnh hoàn chỉnh từ HTML. Giữ terminal này mở.

4. Trên HTML, bật “Khởi tạo”, bấm/kéo đúng nơi robot đang đặt và hướng nhìn thân, bấm gửi AMCL. Hoặc chọn GLOBAL nếu map có đủ đặc trưng. Chờ READY và đối chiếu mũi tên xanh với vị trí/hướng ngoài thực tế.
5. Thêm endpoint ở sàn đã quét, kéo hướng muốn camera/thân quay. Lưu endpoint. Có thể dùng endpoint có sẵn. Sau mỗi sửa, BE/Pi đồng bộ profile mới; không cần sửa file hardware.
6. “Chuẩn bị workflow MOVE” → chọn endpoint gần, đường trống → “Đi tới endpoint bằng robot của phiên”. Theo dõi request/job/step output; không chỉ dựa HTTP200.
7. Hủy bằng nút Hủy yêu cầu; chờ job/Pi xác nhận dừng. Đóng phiên rồi Ctrl+C terminal navigation. Đổi nơi test: chọn map khác và mở phiên mới.

### Log kiểm tra (terminal container khác)

```sh
cd /home/ubuntu/mediassist-mqtt-2026-10-08
tail -n 60 pi/state/stack/worker.log
tail -n 60 pi/state/stack/navigation.log
tail -n 60 pi/state/stack/localization.log
tail -n 40 pi/state/stack/slam-engine.log
timeout 8s ros2 topic echo /mediassist/localization/status --once --full-length
timeout 8s ros2 topic echo /mediassist/safety_state --once --full-length
```

`navigation.log` thuộc quét frontier; nhánh saved-map Nav2 nằm trong `slam.log` và `localization.log`. Không chạy docker từ trong container; lệnh docker chỉ ở HOST.

## 6. API Swagger mới

ADMIN:

| Method | API | Dùng để |
|---|---|---|
| GET/POST | `/api/v1/robots/{robotId}/navigation-session` | Xem/mở phiên map tạm |
| POST | `/api/v1/robots/{robotId}/navigation-session/initialization` | SEED hoặc GLOBAL, revision tăng |
| POST | `/api/v1/robots/{robotId}/navigation-session/close` | Đóng đúng session UUID, chặn nếu job còn treo |
| POST | `/api/v1/maps/{mapId}/navigation-copy` | Bản mới sửa ngưỡng unknown, copy endpoint với ID mới |
| POST | `/api/v1/maps/{mapId}/validate-navigation-point` | Kiểm tra tâm điểm nằm trên ô free thật |
| GET | `/api/v1/robots/{robotId}/snapshot` | Pose đã xác minh, tuổi dữ liệu, trạng thái phiên |

DEVICE có GET `/api/v1/pi/navigation-session`, POST `/api/v1/pi/navigation-session/close`, bundle map và profile qua các API đang có.

POST mở phiên:

```json
{"mapId":"73f7af66-ae21-499f-8209-57801411e9be","durationSec":7200}
```

SEED (dùng đúng sessionId vừa trả về, X/Y/yaw đo/chọn thật):

```json
{"sessionId":"UUID_THAT","mode":"SEED","xM":1.0,"yM":1.0,"yawRad":0.0}
```

GLOBAL: `{"sessionId":"UUID_THAT","mode":"GLOBAL"}`. Đóng: `{"sessionId":"UUID_THAT"}`. Không lấy các tọa độ ví dụ trên làm vị trí thật.

## 7. Kết quả kiểm thử và phần còn cần làm trên robot

Xem `VERIFICATION.md`: unit/integration/schema, replay map và browser đã kiểm tra offline. Không gọi offline test là đã chạy được robot thật. Sau khi Pi có pin cần đo/đối chiếu footprint và TF, chạy một goal ngắn, thử dừng/né vật, hoàn tất quét-lưu rồi nạp lại đúng map. GLOBAL cần kiểm chứng trong môi trường thật; không bảo đảm tự định vị trong phòng đối xứng hoặc map trống.

Tham khảo kỹ thuật Nav2 Humble: [bringup](https://github.com/ros-navigation/navigation2/blob/humble/nav2_bringup/launch/bringup_launch.py), [ObstacleFootprint critic](https://github.com/ros-navigation/navigation2/blob/humble/nav2_dwb_controller/dwb_critics/src/obstacle_footprint.cpp), [map loading](https://github.com/ros-navigation/navigation2/blob/humble/nav2_map_server/src/map_io.cpp). Nội dung source hãng đã đối chiếu trong HIWONDER_BASE_AUDIT.md và bản review trước.
