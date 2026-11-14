# Kế hoạch phục hồi Pi, BE và AI voice

Ngày kiểm tra 13/11/2026, múi giờ Asia/Bangkok.

Tài liệu ghi nhận quá trình phục hồi sau khi cài image Hiwonder. Mục 1–8 là khảo sát và kế hoạch ban đầu. Mục 9 ghi nhận đợt cài source đầu tiên. **Mục 10 ghi trạng thái mới nhất trên Pi `192.168.1.111`.**

## 1. Kết quả đã xác minh

| Thành phần | Kết quả |
| --- | --- |
| Pi SSH | `pi@10.3.91.121` truy cập được, hostname `raspberrypi` |
| Nhận dạng SSH | ED25519 `SHA256:vVDuRogpDFTo5WlsxLCf3i1sLFiKmdeZnSOxGDdXP9k`, khớp các địa chỉ Pi đã lưu |
| Host Pi | Kernel `6.12.47+rpt-rpi-2712`, RAM 7,9 GiB, lúc kiểm tra còn 6,3 GiB available |
| Thẻ đang boot | `/dev/mmcblk0p2`, tổng 57 GiB, dùng 44 GiB, còn 11 GiB, 81% |
| Nhiệt và nguồn | 49,9°C, `throttled=0x0` tại thời điểm đọc |
| Container | `MentorPi`, image `ros:humble`, host network, privileged, PID 1 là `tail`, `init` chưa bật |
| Dịch vụ hãng | `start_node.service` đang active; `wifi.service` đang active |
| Runtime MAR | Thiếu cả `/home/ubuntu/mediassist-mqtt-2026-10-08` và `/home/ubuntu/mar-secrets` trong container |
| ROS và thư viện | Có Humble, workspace Hiwonder, Python 3.10.12, Smac planner, RPP, CycloneDDS, CMake, `arecord`, `aplay` |
| DNS host | Phân giải BE ra `3.0.80.8` |
| DNS container | Vẫn dùng `202.96.134.133` và `202.96.128.86`; truy vấn BE timeout sau 5 giây |
| HTTPS trong container | Chưa kiểm chứng được vì truy vấn tên miền timeout |
| Zombie | Đếm được 5 trên host và 5 trong container. Hai số đếm có thể cùng nhìn một nhóm PID, cần đối chiếu PID namespace trước khi cộng |

Dung lượng hiện tại cho thấy phân vùng `/` còn chỗ ghi. Kết quả này chưa thay thế bài kiểm tra độ tin cậy của thẻ sau các lỗi H2testw trước đó. Cần xác nhận thẻ đang dùng đã qua lượt Write + Verify sạch.

### Dịch vụ cloud

| Endpoint | Quan sát từ laptop |
| --- | --- |
| `https://mediassistrover.duckdns.org/actuator/health` | HTTP 200, `status=UP` |
| `mediassistrover.duckdns.org` | DNS A `3.0.80.8`, TLS xác minh thành công, chứng chỉ tới 01/02/2027 |
| `media.mediassistrover.duckdns.org` | DNS A `3.0.80.8`, TLS xác minh thành công, chứng chỉ tới 06/02/2027 |
| Chứng chỉ media | SAN có `media.mediassistrover.duckdns.org` và `turn.mediassistrover.duckdns.org` |
| `https://media.mediassistrover.duckdns.org/health` | HTTP 200, `state=READY`, wake enabled/configured/loaded |
| AI cloud | Wake `M A`, alias `em a`, `mờ a`; response và intent hiện báo `gpt-6-luna`; knowledge source `LOCAL_FILES` |
| AWS CLI | Profile `mar-admin` đã hết hạn. Chưa kiểm tra trực tiếp env EC2, phiên DB, broker credentials hoặc revision JAR bằng AWS trong lượt này |

Health công khai xác nhận dịch vụ đáp ứng và chứng chỉ hợp lệ. Handoff voice, quyền thiết bị, stream thật và MOVE cần kiểm thử sau khi Pi được phục hồi.

## 2. Chốt bản source để giữ luồng trước đây

| Repo | Local | Remote main đã kiểm tra | Chênh lệch |
| --- | --- | --- | --- |
| `C:\GitHub\RASPPI_MAR` | `ebe5cd6` | `ebe5cd6` | Có 18 file tracked sửa local và `patrol_sensing.py` chưa tracked |
| `C:\GitHub\BE_MAR` | `12e0258` | `cf5110b` | Laptop chậm một commit về vô hiệu hóa phòng/endpoint bị gỡ khỏi bản vẽ đã publish |
| `C:\GitHub\AI_VOICE` | `6dbb75c` | `6dbb75c` | Working tree sạch |

Đã fetch BE để đọc diff. Working tree BE vẫn ở `12e0258`. Trước khi triển khai từ laptop, đưa thay đổi BE tới revision mong muốn bằng fast-forward sau khi xác nhận tình trạng Git, rồi đối chiếu CI/deployed revision.

### Nav2, AMCL và các step

| Nhóm | So sánh main với laptop | Cách đưa vào bản phục hồi |
| --- | --- | --- |
| `navigation_setup.py` | Trùng main | Giữ bộ sinh tham số Nav2 hiện có |
| `tools/run_navigation_session.py` | Trùng main | Giữ pipeline nạp bundle map, profile, RPP, committed path và arrival mode hiện có |
| `localization.py`, `configure_localization.py` | Có patch local | Giữ bản đã hiệu chỉnh phục hồi chất lượng định vị; transform tolerance local là 0,5 giây |
| `cargo_recovery.py`, `ros_adapter.py` | Có patch local | Giữ logic tìm hướng thoát kẹt, kiểm tra footprint và căn hướng khi gắn thùng |
| `navigation_guidance.py`, `speech.py` | Có patch local | Giữ logic phát chỉ dẫn và callback bắt đầu playback |
| `staff_qr.py`, `step_contract.py`, `step_worker.py`, `worker_config.py` và helper delivery | Có patch local | Phục hồi đồng bộ quyền QR, commissioning có phạm vi và kết quả step |
| `navigation_zones.py`, `worker_adapter.py` | Có patch local | Đối chiếu cả dependency và profile khi đóng gói source |
| `pi/schemas/step_types.json` | Trùng main | Vẫn 11 StepType |

11 StepType hiện có gồm `MOVE`, `INTERACT`, `DATA_SERVICE`, `AUTHENTICATE`, `CONTROL_HARDWARE`, `HANDLE_PAYLOAD`, `EXECUTE_SENSING`, `NOTIFY`, `WAIT`, `CHECK_CONDITION`, `SET_CONTEXT`. Pi worker nhận các loại được cấu hình tại `enabled_step_types`; các bước điều phối còn lại thuộc executor tương ứng trên BE.

Schema MOVE vẫn yêu cầu tham số tốc độ theo contract. BE phải resolve tốc độ mặc định trước khi dispatch khi request ban đầu bỏ trống giá trị. Mục tiêu vận hành đã chốt trước đây là 0,35 m/s cho escort/delivery, còn tốc độ thực tế chịu giới hạn của hardware profile và zone.

Lượt vá cuối có biên nhận bốn file production và stack đã dừng, nhưng cấu hình hardware/worker vận hành cũ nằm ngoài Git. Vì thiếu profile cũ, chưa thể xác nhận mọi tốc độ, khoảng dừng, watchdog và hiệu chỉnh servo giống hoàn toàn phiên trước. Việc khôi phục các giá trị này là đầu việc riêng ở mục 5.

`patrol_sensing.py` là phần mở rộng đang chuẩn bị. Khi đóng gói các file có import phụ thuộc tới module này, phải đưa đủ module và giữ phần tính năng mới chờ nghiệm thu sau luồng MOVE/voice/delivery cơ bản.

## 3. DNS, HTTPS, TLS và CA cần cấu hình gì

| Kết nối | Địa chỉ/cấu hình cần dùng | Ảnh hưởng của IP Pi mới |
| --- | --- | --- |
| SSH | `10.3.91.121` | Cập nhật SSH target |
| Camera LAN | `http://10.3.91.121:8080/stream?topic=%2Fascamera%2Fcamera_publisher%2Frgb0%2Fimage&type=mjpeg&width=640&height=480&quality=60` | URL dự kiến sau khi camera stack được phục hồi |
| Pi → BE | `https://mediassistrover.duckdns.org` | Phục hồi URL và CA trust của client |
| Pi → AI cloud | `https://media.mediassistrover.duckdns.org` | Phục hồi AI token và base URL |
| Pi → WebRTC | `wss://media.mediassistrover.duckdns.org` | Phục hồi media config/token flow |
| MQTT | Host broker, port và CA lấy từ cấu hình production đang chạy | Cần phục hồi credentials và chứng chỉ trust đúng broker |
| MySQL | BE truy cập MySQL theo env EC2 | Pi trao đổi với BE qua HTTPS/MQTT |

Các domain công khai và chứng chỉ đang hợp lệ. Việc Pi đổi IP LAN không tự tạo yêu cầu cấp lại chứng chỉ BE, AI hay media. Đường MQTT, HTTPS BE/AI và CA cho DB là các cấu hình trust riêng, cần đối chiếu đúng phía sử dụng.

Với HTTPS BE/AI công khai, kiểm tra từ container bằng CA hệ thống, dự kiến `/etc/ssl/certs/ca-certificates.crt`. Chứng chỉ riêng chỉ phân phối lại khi lựa chọn endpoint hoặc chuỗi CA thực sự thay đổi. Private key của server thuộc máy server.

### Ưu tiên sửa DNS container

Sau khi source đã được chuyển vào container, dùng các helper production hiện có trên Pi host.

```bash
# Trên host Pi, bên ngoài MentorPi, sau bước phục hồi source
sudo docker cp MentorPi:/home/ubuntu/mediassist-mqtt-2026-10-08/tools/repair_pi_container_dns.py /tmp/mar-repair-dns.py
sudo python3 /tmp/mar-repair-dns.py --container MentorPi --broker mediassistrover.duckdns.org
```

Đọc kết quả check-only rồi thực hiện `--apply` khi phục hồi. Sau khi worker config chứa hostname MQTT đúng, cài `tools/install_container_dns_sync.py` cùng `repair_pi_container_dns.py` bằng `--apply` trên host để DNS theo sự kiện NetworkManager/Docker. Helper được viết cho container host-network đang dùng.

Nghiệm thu từ chính container phải đạt phân giải BE, AI, broker, rồi xác minh HTTPS và TLS MQTT bằng hostname thật. Lỗi DNS cần được xử lý trước khi diễn giải lỗi token hoặc lỗi chứng chỉ.

### Nếu chủ động chuyển AI về laptop

Laptop đang ở `10.3.90.235`. Local `.env` có cấu hình AI và khóa đã điền, nhưng chưa có listener 7861 lúc kiểm tra. Chứng chỉ local trong `%LOCALAPPDATA%\MediAssistRover\voice-ai` chưa có SAN cho IP này.

Lượt kiểm kê bổ sung đã tìm thấy `ca.key` và xác minh public key khớp `ca.pem`; `server.key` cũng khớp `server.pem`. Helper `create_mapping_tls.py` lại tìm tên `ca-key.pem`, nên việc thiếu tên file đó chưa có nghĩa mất CA key. Khi dùng local, cần cho helper đọc đúng khóa đã xác minh rồi cấp lại leaf với SAN của laptop hiện tại. Private CA key và server key giữ trên laptop/server; Pi chỉ nhận public CA. Chỉ đổi `voice-ai.json.base_url` sang `https://10.3.90.235:7861` sau khi kiểm tra chứng chỉ và firewall. Hiện chọn cloud AI giúp dùng domain ổn định đã kiểm tra.

### Kiểm kê MQTT và khóa đã lưu trên laptop

| Tài nguyên | Tình trạng đã kiểm tra | Phần cần đưa lại lên Pi |
| --- | --- | --- |
| `Downloads/emqx-ca.crt`, `Downloads/emqxsl-ca.crt` | CA DigiCert Global Root G2, hạn tới 2038 | Public CA broker nếu dùng `tls_ca_file` riêng |
| Broker trong `.env` laptop | `e17e61cf.ala.us-east-1.emqxsl.com:8883`; TLS 1.3 xác minh thành công bằng CA đã lưu, leaf tới 12/01/2027 | Đối chiếu broker production trước khi dựng worker; chưa thử MQTT login |
| `Downloads/mar-ec2-key.pem` và bản `secrets/aws/ec2-key.pem` cũ | Đọc được private key RSA hợp lệ | Dùng để quản trị SSH nếu máy đích còn chấp nhận; MQTT hiện không sử dụng chúng |
| `%USERPROFILE%/.ssh/mar-remote` | Khóa SSH mã hóa bằng passphrase | Giữ trên laptop, dùng cho quản trị SSH |
| Bộ `voice-ai/ca.pem`, `ca.key`, `server.pem`, `server.key` | Hai cặp khóa/chứng chỉ khớp nhau | Local AI chỉ cần chuyển public CA tới Pi |
| `.env` BE/AI trên laptop | Có MQTT password của BE, OpenAI key và AI bearer token | Chưa tìm thấy bản sao riêng password của MQTT user Pi và BE device secret; AI token cần đối chiếu cloud trước khi dùng lại |

Contract runtime là `MAR-STEP-2`, protocol `1.0`, transport MQTT 5 qua TLS. Pi xác thực bằng username/password của broker, username là robot UUID. Transport hiện không nạp client certificate/private key. `HELLO_REJECTED` là BE từ chối ở tầng ứng dụng sau khi đã kết nối broker, cần đọc nguyên nhân HELLO thay vì suy ra lỗi CA hoặc DNS.

File `.env` BE tại `C:\GitHub\BE_MAR` hiện có allowlist `WAIT,EXECUTE_SENSING,CONTROL_HARDWARE`, thiếu `MOVE` và `INTERACT`. Đây là cấu hình local, chưa phải bằng chứng env EC2 đang chạy. Khi phục hồi, đối chiếu env production và cho phép đúng các step cần dùng. Không khởi động thêm BE local cùng robot để tránh có nhiều bộ điều phối.

## 4. Thứ tự phục hồi runtime

### A. Xác minh nền Pi

- [x] SSH, RAM, dung lượng `/`, nhiệt và throttling đã đọc được.
- [x] Đã xác nhận image hãng có Python 3.10, ROS Humble, Smac, RPP và CycloneDDS.
- [ ] Xác nhận thẻ đang dùng đã qua kiểm tra ghi/đọc sau các lỗi trước đây.
- [ ] Đối chiếu cảnh báo duplicate swapfile trong `/etc/fstab` rồi sửa đúng entry trùng khi thực hiện phục hồi.
- [ ] Xác định PID cha của zombie. PID 1 hiện là `tail`; lên phương án init/reaper đúng và giữ đủ host network, device, mounts của container khi cần tái tạo.
- [ ] Khi phục hồi MAR, xác định service hãng đang sở hữu bringup rồi dừng/disable đúng service để có một bộ driver. Giữ dịch vụ Wi-Fi hoạt động.

### B. Chuyển source production đã chốt

Đích trong `MentorPi` là `/home/ubuntu/mediassist-mqtt-2026-10-08`. Nguồn lấy từ working tree đã audit ở `C:\GitHub\RASPPI_MAR\PI_MAR`, gồm các patch production đã chốt. Tránh chỉ checkout remote main rồi bỏ mất các sửa local nêu ở mục 2.

Đóng gói source cần loại `.git`, virtualenv Windows, `__pycache__`, log và test artifacts. Các test có sẵn chạy tại nơi phù hợp khi nghiệm thu, còn payload đưa lên Pi tập trung vào runtime và tool production. Trước khi chuyển, lập danh sách file và hash để so sánh lại trên Pi.

### C. Tạo môi trường Python và native plugin

Chạy trong container với user `ubuntu`, tại thư mục dự án sau khi source đã có.

```bash
python3 -V
python3 -m venv --system-site-packages pi/.venv
pi/.venv/bin/python -m pip install -r pi/requirements.txt
bash tools/build_navigation_bt.sh
```

`PI_MAR/offline` hiện vắng trong checkout, nên bộ `install_pi_offline.sh` chưa đủ dependency để tự cài offline. Native plugin cần tạo `pi/native/navigation_bt/build/libmar_commit_navigation_path_bt_node.so`. Script build đã có kiểm tra CTest.

WebRTC dùng dependency tách biệt theo cơ chế media hiện có.

```bash
pi/.venv/bin/python -m pip install --target pi/.venv/media-site-packages -r pi/requirements-media.txt
```

Kiểm tra OpenCV/cv_bridge, NumPy phù hợp ROS Humble, ALSA, ZBar/pyzbar và LiveKit trước khi bật luồng tương ứng.

## 5. Phục hồi worker config và hardware profile

Thiếu private config là phần phụ thuộc chính cần hoàn thành. Tìm profile vận hành và bản sao secret trước. Nếu không còn, dựng lại profile từ schema và kiểm tra phần cứng thực tế trước khi bật các cờ commissioned.

| Mục | Cần phục hồi hoặc kiểm chứng |
| --- | --- |
| Robot identity | ID trước đây `0e3f1083-2546-4f45-8004-5eda5cb75a6f`; xác nhận qua BE production |
| Worker nền | `pi/config/mqtt.worker.json`, tham chiếu `hardware_profile` bằng đường dẫn Linux |
| Journal | `state_path` và `legacy_state_path` là hai SQLite khác nhau |
| MQTT | `mqtt.host`, `port`, `username=robot_id`, `password_file`, `tls_ca_file`, client/session identity |
| BE device auth | `mapping.robot_code`, `device_secret_file`, `upload_token_file`, URL và CA upload |
| AI voice | `/home/ubuntu/mar-secrets/voice-ai.json`, `base_url`, `token_file`, `ca_file`; token khớp AI cloud |
| Speech | `speech.synthesize_url` và CA/token đúng BE; bật `INTERACT` ở cả Pi và rollout BE |
| QR/servo | `CART_MAIN`, profile servo thật, `STAFF_VERIFIED` và quyền commissioning được cấu hình có phạm vi |
| Geometry | Thân 212 × 171 mm, tâm footprint theo TF thật; cargo gắn cứng nhô sau 100 mm, rộng 150 mm |
| Mapping và navigation | Mapping bare, navigation chọn `--load-profile cargo` khi gắn thùng |
| Tốc độ | Mục tiêu MOVE 0,35 m/s; phục hồi riêng tốc độ góc và ngưỡng giảm/dừng từ profile đã chốt |
| Arrival và AMCL | Giữ path tracking, arrival mode, tolerance và các ngưỡng chất lượng đã xác minh |
| Camera/media | Giữ driver `ascamera` Hiwonder, đúng ROS topic; QR và WebRTC là consumer của cùng driver |

Đã tìm các tên cấu hình chuẩn trong những thư mục laptop được kiểm tra và chưa thấy worker/hardware profile vận hành hiện hành. Các `site.local.json` cũ từ tháng 10 chỉ là nguồn đối chiếu, cần xác minh schema và calibration trước khi dùng.

### Secret bị mất trên thẻ

BE lưu device secret dạng BCrypt. Raw secret cũ cần lấy từ bản sao riêng, hoặc cấp lại qua `POST /api/v1/robots/{id}/rotate-secret` cho robot hiện có sau khi chốt trạng thái job/session. `provision-secret` dùng khi robot chưa từng được cấp secret.

Sau đó kiểm tra `POST /api/v1/pi/auth/token`, lưu secret vào file private trên Pi và hạn chế quyền truy cập. MQTT password, BE device secret và AI bearer token là ba loại thông tin khác nhau. Tài liệu này chỉ ghi tên cấu hình.

`tools/configure_mqtt.py` dựng worker nền sau khi hardware profile hợp lệ. Cần đặt chủ động các step được hỗ trợ. Helper mặc định có heartbeat 3 giây, ACK timeout 15 giây, HELLO timeout 20 giây và BE timeout 15 giây; đối chiếu với BE trước khi áp dụng.

Worker cấp phiên tại `pi/state/navigation-sessions/.../worker.json` sẽ được launcher sinh lại từ worker nền và descriptor BE. Sau cài thẻ mới, tạo session mới để khớp process instance và map hash.

### Helper dùng khi thực hiện

- `configure_be_origin.py` có check-only, yêu cầu worker nền đã hợp lệ và device secret có sẵn. Dùng origin cloud và CA hệ thống để kiểm health + device auth, sau đó `--apply` khi stack dừng.
- `configure_move_profile.py --speed .35 --apply` phục hồi trần MOVE trong giới hạn phần cứng; đọc dry-run trước.
- `configure_navigation_geometry.py --rear-extension .10 --cargo-width .15 --default-profile cargo --apply` áp dụng geometry sau khi xác minh tâm và kích thước.
- `configure_us100.py --mode optional` để đối chiếu cấu hình cảm biến hiện có.
- `configure_camera_backend.py --backend web_video_server` kiểm cấu hình camera trước khi `--apply`.
- Bản local `configure_staff_delivery.py --allow-commissioning-unlock` hỗ trợ QR và test servo được BE cấp quyền trong cùng profile. Đọc dry-run rồi mới dùng `--apply`.
- `configure_tts.py` ghi cấu hình ngay, thực hiện trong giai đoạn stack dừng. Tốc độ giọng mong muốn trước đây 1,15× cần kiểm lại env BE, vì source BE hiện có default 1,25×.
- `configure_media.py --url wss://media.mediassistrover.duckdns.org --enable` kiểm dependency trước khi `--apply`.

## 6. Kiểm tra BE, job và hợp đồng MQTT

Các API sau hiện có trong OpenAPI của BE deploy.

| API | Mục đích |
| --- | --- |
| `POST /api/v1/pi/auth/token` | Device token |
| `POST /api/v1/pi/navigation-session/plan` | Plan đã được Pi tính và xác nhận |
| `GET/POST /api/v1/robots/{robotId}/navigation-session` | Đọc/tạo phiên map |
| `POST /api/v1/test-workflows/run` | Workflow test bằng StepType thật |
| `GET /api/v1/pi/media-session` | Thông tin phiên media của Pi |
| `POST /api/v1/jobs/recovery/bulk-close` | Xử lý recovery có dry-run |
| `POST /api/v1/robots/{id}/rotate-secret` | Cấp lại device secret |

Đọc trạng thái robot/job trước khi chốt session cũ. Dùng API cancel/recovery và bằng chứng dừng phù hợp để đóng job còn dang dở sau lần mất runtime. Sau khi hoàn tất, mở session mới trên map thực tế.

BE phải giữ MQTT, dispatch và quản lý đúng robot ID. Kiểm `GET /api/v1/admin/mqtt-rollout` với tài khoản ADMIN. Kiểm riêng `MQTT_ENABLED_STEP_TYPES` có `INTERACT` khi sử dụng phát loa.

Hai luồng escort bằng voice và delivery QR tiếp tục resolve cùng loại MOVE. Khác biệt nghiệp vụ nằm ở endpoint, tốc độ, payload và các step tiếp theo. Trong một session đã READY, AMCL tiếp tục cập nhật khi robot di chuyển; mỗi job mới dùng lại session hợp lệ và nhận goal riêng. Việc tính lại đường quanh vật cản do Nav2/Pi đảm nhiệm. Đối chiếu ACK/job result theo contract hiện có.

Revision cloud, env thực tế, broker hostname và các job đang mở vẫn cần quyền ADMIN BE hoặc phiên AWS hợp lệ để xác nhận. Health và OpenAPI công khai chưa cung cấp các giá trị private đó.

### Đối chiếu deploy và CI khi có quyền cloud

Trong PowerShell, `aws login --profile mar-admin --region ap-southeast-1` làm mới phiên CLI, sau đó dùng `aws sts get-caller-identity --profile mar-admin --region ap-southeast-1` để xác nhận account `179897609252`.

- BE CI tại `.github/workflows/ci.yml` dùng GitHub OIDC, S3 và SSM. Đối chiếu `AWS_DEPLOY_ROLE_ARN`, `MAR_DEPLOY_BUCKET`, `MAR_EC2_INSTANCE_ID`, kết quả workflow và commit JAR đang chạy.
- AI CI tại `.github/workflows/voice-ci.yml` phụ thuộc `AI_VOICE_DEPLOY_ROLE_ARN`, `AI_VOICE_ARTIFACT_BUCKET`, `AI_VOICE_MEDIA_INSTANCE_ID`.
- AI service dự kiến `mar-voice-ai.service`, env `/etc/mediassistrover/voice-ai.env`, runtime `/opt/mar-voice/current`, data `/var/lib/mar-voice/data`. Kiểm tra đúng media instance trước khi đọc trạng thái service.
- BE và media edge dùng HTTPS/WSS ở public domain, upstream HTTP private giữa các service theo cấu hình deploy đang có. Kiểm tra DNS/TLS công khai trước khi thay env.
- AI cloud hiện báo `knowledge_source=LOCAL_FILES`. Muốn đồng bộ tài liệu bệnh viện từ BE cần thêm đúng `AI_HOSPITAL_ID` và xác minh published catalog. Tính năng này thực hiện sau khôi phục luồng voice cơ bản.

## 7. Nghiệm thu theo thứ tự

1. `zsh tools/robot.zsh inspect`, rồi `zsh tools/robot.zsh prepare`. Kiểm `PREPARE_READY` và dừng fully bằng Ctrl+C.
2. Kiểm DNS từ container, HTTPS BE/AI, device token và MQTT HELLO. Cần có `BE MQTT session ready`.
3. Kiểm camera LAN, frame timestamp, QR decoder và media publisher cùng dùng một driver camera.
4. Kiểm micro/loa bằng tool có sẵn, rồi STT/TTS. Voice listener chạy do supervisor quản lý.
5. Chọn đúng map và tải bundle PGM/YAML/hash từ BE. Mở session mới, nạp pose ban đầu và chờ READY.
6. Thử MOVE ngắn, một tuyến nhiều endpoint, rồi vật cản tạm thời và zone. Đo thời gian gửi request, nhận command, Nav2 nhận goal và kết quả cuối.
7. Thử workflow MOVE → WAIT → SPEAK và workflow delivery QR tại hai đầu. Kiểm QR grant đúng job/step và servo trả kết quả.
8. Thử WebRTC/SOS bằng staff khác mạng sau khi publisher Pi hoạt động.
9. Kiểm cancel, step lỗi, job kế tiếp trong queue, đóng session và thu hồi process con. Chốt trạng thái BE trước khi tắt robot.

Lệnh session dự kiến sau khi các bước phục hồi hoàn tất.

```bash
cd /home/ubuntu/mediassist-mqtt-2026-10-08
zsh tools/navigation_session.zsh --session-id NEW_SESSION_ID --load-profile cargo --voice-chat
```

Sau khi các dòng STOP hoàn tất, thực hiện `sudo poweroff` ở host Pi rồi mới ngắt nguồn.

## 8. Điều kiện còn thiếu để xác nhận phục hồi tương đương bản cũ

- [ ] Worker config nền và hardware profile đúng schema, calibration và tốc độ.
- [ ] BE device secret, MQTT password, AI token và trust paths hợp lệ.
- [ ] DNS container đã theo host và phân giải broker/BE/AI.
- [ ] Bộ source gồm đúng các patch production đã chốt, hash trên Pi khớp laptop.
- [ ] Plugin native đã build và virtualenv có đủ dependency.
- [ ] Service hãng và supervisor MAR có quyền sở hữu thiết bị rõ ràng.
- [ ] Job/session cũ đã được đối chiếu và chốt qua API.
- [ ] Nghiệm thu trên robot cho MOVE, AMCL, multi-endpoint, voice, QR, WebRTC và cancel.

## Nguồn đối chiếu

- [Pi runtime và cấu hình riêng](../README.md)
- [Sinh worker config](../tools/configure_mqtt.py)
- [Launcher navigation session](../tools/run_navigation_session.py)
- [Tham số Nav2](../pi/mediassist/navigation_setup.py)
- [Chất lượng định vị](../pi/mediassist/localization.py)
- [DNS container](../tools/repair_pi_container_dns.py)
- [Đồng bộ DNS theo sự kiện](../tools/install_container_dns_sync.py)
- [BE origin và CA](../tools/configure_be_origin.py)
- [Giới hạn MOVE](../tools/configure_move_profile.py)
- [BE source](C:/GitHub/BE_MAR)
- [AI voice source](C:/GitHub/AI_VOICE)

## 9. Cập nhật phục hồi sau khi được phép cài

### Đã xác minh lại production

- SSH tới `3.0.80.8` bằng khóa quản trị đã lưu thành công. IMDS xác nhận account `179897609252`, instance `i-02f31d6c01c9978d6`, Singapore, private IP `172.31.42.120`.
- Domain BE vẫn phân giải `3.0.80.8`. SSH host `56.10.141.8` trong thông tin connection người dùng gửi thêm đang timeout; chưa chuyển DB hay URL sang host này.
- MySQL production xác nhận robot UUID giữ nguyên, `robotCode=mar-v1`, trạng thái lúc đọc là `OFFLINE`.
- Không có job chưa kết thúc cho robot. Navigation session gần nhất đã đóng ngày 11/11/2026.
- MQTT login thử bằng client ID kiểm tra riêng thành công cho cả user BE và user robot UUID. Không publish command, không dùng client ID của worker thật.
- Env BE đang bật MQTT/dispatch, allowlist có đủ `MOVE,CONTROL_HARDWARE,EXECUTE_SENSING,WAIT,INTERACT`, STT/TTS bật.
- `OPENAI_TTS_SPEED` trong env đang là `1.0`; chưa đối chiếu override runtime trong DB và chưa thay tốc độ BE ở lượt này.
- API ADMIN hoạt động. Device secret đã rotate qua API cho robot hiện có, sau đó lấy DEVICE JWT thành công. Không sửa hash trực tiếp trong DB.
- AI bearer token đang lưu trên laptop đã qua bước xác thực của endpoint cloud wake. Chỉ kiểm tra auth bằng request sai định dạng, không thu âm hoặc tạo hội thoại.

Device secret mới có bản lưu root-only tại `/etc/mediassistrover/devices/0e3f1083-2546-4f45-8004-5eda5cb75a6f.json` trên BE EC2. Bộ cấu hình riêng trên laptop nằm tại `%LOCALAPPDATA%\MediAssistRover\pi-production`, đã giới hạn ACL cho user Windows hiện tại. Tất cả secret giữ ngoài Git và tài liệu này.

### Đã cài trên Pi trước khi người dùng tạm sửa thiết bị

- 157 file source production đã chuyển vào `/home/ubuntu/mediassist-mqtt-2026-10-08`, từng file được so SHA-256 với manifest. Không có file trùng nội dung, test, log, backup, Windows virtualenv hoặc Git trong payload.
- Private CA MQTT, MQTT password, BE device secret/token và AI token đã đặt trong `/home/ubuntu/mar-secrets`, quyền thư mục 0700 và file 0600.
- Sau lần Pi khởi động lại, container DNS đã trùng host và phân giải broker thành công. Đã cài hook systemd/NetworkManager để đồng bộ lại khi đổi mạng.
- Đã bổ sung `python3.10-venv`, `python3-pyzbar`; tạo venv dùng ROS system packages và cài requirements worker.
- Đã cài dependency WebRTC vào `pi/.venv/media-site-packages`, tách khỏi môi trường ROS mặc định.
- Native `libmar_commit_navigation_path_bt_node.so` đã build thành công bằng `MAR_BUILD_TESTING=OFF`; không chuyển source test native lên Pi.
- Chưa khởi động stack MAR, chưa gửi MOVE, chưa phát PWM servo.

APT có cảnh báo khóa ROS mirror cũ hết hạn; hai gói Ubuntu cần thiết vẫn cài thành công. Chưa thay trust policy hoặc bỏ kiểm tra chữ ký. Pip báo các dependency AI mẫu của image hãng không đủ metadata OpenCV và khác protobuf; cần kiểm import ROS/OpenCV và media trong môi trường tách biệt khi tiếp tục, trước khi bật stack.

### Bộ cấu hình đã chuẩn bị trên laptop, chưa ghi lên Pi

`site.production.json`, `mqtt.worker.json`, `voice-ai.json` đã được tạo trong thư mục private nói trên. Hardware/autonomy/camera/localization/geometry đã được kiểm tra bằng validator trong RAM. Chưa chạy `worker_config.load` trên Pi cho bộ file này.

| Mục | Giá trị chuẩn bị |
| --- | --- |
| Footprint bare | front 0,109; rear 0,104; left/right 0,086 m |
| Footprint cargo | rear tăng một lần lên 0,204 m; thùng rộng 0,15 m |
| MOVE cap | 0,35 m/s; yaw 1,2 rad/s |
| Manual | 0,25 m/s; yaw 2,0 rad/s |
| Mapping cap | 0,18 m/s cho giai đoạn phục hồi; đây là lựa chọn tạm vì thiếu trần private gần nhất |
| AMCL match | Bootstrap và tracking 0,55; giữ các guard TF/scan/pose khác |
| Camera | Driver Hiwonder, relay Python cổng 8080, 640×480, 15 FPS, JPEG quality 60 |
| Voice | Cloud HTTPS, wake local-AI `M A`, STT auto Việt–Anh, actions bật, MOVE 0,35 m/s |
| QR/servo | `CART_MAIN`, `STAFF_VERIFIED`; cần áp dụng bản sửa callback/SDK và nghiệm thu trước khi bật commissioning |
| Commissioning | Các cờ motor/watchdog/geometry/servo đang false trong candidate; chưa tạo bằng chứng stop-test mới |

Base worker có 4 loại `EXECUTE_SENSING,WAIT,CONTROL_HARDWARE,INTERACT`. Launcher sẽ thêm MOVE sau khi tải descriptor/map thật cho session mới. Không tái sử dụng session cũ.

Source đã sửa thêm việc launcher ghi đè localization để giữ ngưỡng đã cấu hình khi tạo runtime. Wrapper ROS đặt `TMPDIR` trong thư mục private dưới `/dev/shm` cho các child process. Hai thay đổi phục hồi cấu hình này không thay thuật toán planner/controller.

### Dọn payload trong lượt vá tiếp theo

Đã rà tham chiếu và xác định 7 tool chỉ phục vụ export hoặc preset/import cũ, có thể giữ ở laptop và bỏ khỏi Pi. Payload kế tiếp còn 150 file, gồm 49 tool và 69 module runtime.

- `tools/export_contract.py`, `tools/export_mqtt_contract.py`
- `tools/install_pi_offline.sh`
- `tools/configure_demo.py`, `tools/configure_bare_mecanum.py`
- `tools/install_be_map.py`, `tools/import_be_endpoints.py`

`pending-deployment.json` trong thư mục private ghi rõ các file giữ lại, SHA hiện tại của 7 file trước khi xóa và đích của ba config. **Chưa xóa 7 file trên Pi.** Các tool này vẫn được giữ trong source laptop.

### Phần còn lại khi người dùng báo thiết bị sẵn sàng

1. Kết nối lại đúng IP Pi và kiểm tra host key. Đọc camera USB và ALSA; lượt kiểm tra gần nhất chỉ có tay cầm/serial/HDMI audio, người dùng xác nhận chưa cắm đủ thiết bị.
2. So manifest hiện tại rồi dọn đúng 7 tool đã xác định. Ghi ba config candidate với quyền private và chạy loader thật.
3. Kiểm import ROS, NumPy, cv_bridge, pyzbar, LiveKit; preflight callback PWM của image mới rồi áp dụng bản sửa production.
4. Chốt commissioning bằng bằng chứng phù hợp trước khi cho phép chuyển động/servo. Không dùng việc compile thành công làm bằng chứng phần cứng đã test.
5. Đối chiếu service hãng đang sở hữu bringup, giữ Wi-Fi và tránh chạy trùng. PID 1 container vẫn là `tail`; zombie cũ và cấu hình init chưa được xử lý trong lượt này.
6. Kiểm HTTPS/device refresh/MQTT HELLO từ chính Pi, sau đó camera, audio và WebRTC. Tạo session mới trên đúng map thực tế khi thử navigation.
7. Cập nhật manifest/config backup ngoài Git sau nghiệm thu. Hiện bộ runtime/config chưa được nghiệm thu chuyển động, QR hoặc voice trên phần cứng.

## 10. Đã cài cấu hình và giới hạn log trên Pi `192.168.1.111`

- Đã ghi và xác minh `pi/config/mqtt.worker.json`, `pi/config/site.production.json` và `/home/ubuntu/mar-secrets/voice-ai.json`. Quyền file cấu hình và secret là 0600. Profile cargo, MOVE 0,35 m/s và AMCL match 55% khớp bộ cấu hình đã chuẩn bị.
- Đã đối chiếu hash từng file và bỏ 7 tool cũ cùng 2 JSON mẫu. Payload còn 148 file source, 1.032.356 byte. Manifest đã cập nhật trên Pi và trong thư mục private laptop.
- Từ Pi đã lấy được BE device token mới. MQTT CONNACK và SUBACK cho `commands`, `event-acks`, `mapping/control` đều thành công bằng client chẩn đoán. AI cloud báo `READY`. API speech trả STT/TTS configured và tốc độ TTS hiệu lực 1,15×. Chưa chạy worker thật để kiểm HELLO.
- Supervisor xoay log văn bản sau 15 phút hoặc 4 MiB. Đoạn log đã đóng hết hạn 12 giờ tính theo lần ghi cuối. Cơ chế kiểm tra định kỳ khi stack chạy và dọn khi stack khởi động lại. Mỗi thành phần giữ tối đa khoảng 32 MiB gồm file hiện tại và các đoạn đã xoay. Journal, map và log hệ thống nằm ngoài chính sách này. Đã kiểm trên Pi bằng dữ liệu tạm trong `/dev/shm`, không tạo file test trên thẻ.
- Đã kiểm import ROS, OpenCV, ZBar và LiveKit. Helper QR đã nhận diện được cách cài controller dạng editable trên image này và preflight khớp hai file vendor. Vendor bringup vẫn chạy, bản sửa PWM chưa được áp dụng vào driver.

Trạng thái ở thời điểm mục 10 đã được cập nhật tiếp trong mục 11 dưới đây.

## 11. Đã hoàn tất cấu hình thử servo đứng yên

- Pi `192.168.1.111`: một supervisor MAR đang chạy ở chế độ base/mapping đứng yên. Worker đã nhận HELLO ACK thật; BE HTTPS báo `UP`, robot `IDLE`, `mqttConnected=true`, `heartbeatFresh=true`, `dispatchReady=true`, không có failure reason hoặc job chưa kết thúc tại lúc kiểm tra.
- Đã bổ sung đúng tài nguyên runtime `handoff/mapping-live.schema.json`. Tổng payload là **149 file production**; so SHA-256 toàn bộ không có sai lệch. Các import runtime, catalog và native BT plugin đã kiểm tra trên Pi. Không đưa file test lên Pi.
- Đã sửa trạng thái mapping trên thẻ mới: object rỗng được khai báo `session_open=false` trước khi thêm diagnostics. Phiên thực sự đang mở vẫn giữ logic recovery; không đổi planner/controller/AMCL. Bản sửa giải quyết trường hợp BE hiểu nhầm robot vừa khởi động là đang có phiên quét.
- Hai file driver RRC hãng đã áp dụng callback và PWM readback có timeout. Đọc ROS service S6/ID4 thành công, giá trị `1500`. Chưa phát lệnh đổi PWM trong lượt cài này; giá trị controller không thay thế cảm biến cửa cơ khí.
- Servo MG996R: mở `2400`, đóng `1500`, duration `1,0 s` là thời gian di chuyển servo. Thời gian giữ cửa mở do WAIT/VERIFY trong workflow quyết định.
- Pi giữ cấu hình `STAFF_VERIFIED`, đồng thời bật `allow_commissioning_unlock=true`. BE deploy bật `MQTT_ALLOW_COMMISSIONING_UNLOCK=true`; API test-configuration xác nhận Pi quảng bá capability commissioning. Trong base mapping, staff QR unlock vẫn chưa quảng bá (`piUnlockPolicy=DISABLED`); ADMIN servo-test dùng capability riêng và không cần map/camera. QR delivery cần phiên navigation hợp lệ.
- Đã cài `python3-lgpio` và thêm user container `ubuntu` vào nhóm GPIO 993 bằng helper có sẵn. US100 node nhận RP1, BCM17/18 và BCM22/23; đã có status `OK` cùng khoảng cách. Chưa nghiệm thu khoảng cách/stop-test; US100 vẫn là optional, không tự bật commissioning bánh.
- Camera giữ một driver Hiwonder `ascamera`; `/scan` cũng chỉ có một publisher. Đo tại ROS: camera khoảng `12,4 Hz`, lidar khoảng `10,1 Hz`. Stream native `ros_compressed` HTTP 200; đo sau khi stream ổn định khoảng `12,6 FPS` cả loopback và Wi-Fi.
- Link LAN hiện tại: `http://192.168.1.111:8080/stream?topic=/ascamera/camera_publisher/rgb0/image&type=ros_compressed`. WebRTC vẫn qua `https://mediassistrover.duckdns.org/media-call.html`; chưa tạo cuộc gọi SOS mới để nghiệm thu đầu cuối trong lượt này.
- Micro/loa USB đã nhận ALSA; thu một giây ra `/dev/null` và phát kiểm tra đã qua. Hội thoại wake chỉ tự chạy trong launcher navigation có `--voice-chat`; base đứng yên hiện tại hỗ trợ các job thu/phát, chưa mở wake listener.
- Các cờ commissioning bánh/watchdog/footprint vẫn false. Chưa gửi MOVE, chưa khẳng định navigation/QR delivery đã nghiệm thu. Cần nghiệm thu phần cứng và mở session đúng map thực tế trước khi thử di chuyển.
- Bản sao cấu hình private và manifest đã đồng bộ tại `%LOCALAPPDATA%\MediAssistRover\pi-production`; không chứa trong Git. Chưa commit/push các thay đổi local.

### Thử ngay qua BE đã deploy

1. Mở `https://mediassistrover.duckdns.org/mapping.html#robotTestWorkflow`, dùng token ADMIN và robot `0e3f1083-2546-4f45-8004-5eda5cb75a6f`.
2. Xóa danh sách step cũ, kiểm tra cấu hình Pi/BE, thêm một step đóng servo rồi chạy workflow. Quan sát cơ khí và chờ job kết thúc trước khi thử mở.
3. API tương đương: `POST /api/v1/robots/{robotId}/servo-test`, header `Idempotency-Key` mới cho mỗi thao tác, body `{"action":"CLOSE","compartmentId":"CART_MAIN"}`; đổi thành `OPEN` khi cần.
4. Dừng stack đang chạy từ terminal container bằng `zsh tools/robot.zsh stop`, rồi chờ các dòng STOP trong `pi/state/stack/supervisor.log`. Không chạy thêm stack thứ hai.

## 12. Chẩn đoán điều khiển HTML trên Pi 192.168.137.137

- Worker ghi nhiều lần `Mapping control rejected: code=MOTION_NOT_COMMISSIONED` trong lượt quét job `72a85730-ea9b-46b4-8477-df1ad7d6380a`. Ba cờ motion/footprint/watchdog vẫn false theo cấu hình phục hồi đứng yên.
- ACK `PI_ACCEPTED` chỉ xác nhận nhận lease. STOP với vận tốc bằng 0 vẫn được nhận khi chưa commissioning; receipt hiện không chứa vận tốc nên chưa thể kết luận gói `1f5a6e24…` là lệnh đi hay dừng chỉ từ JSON người dùng gửi.
- Trang HTML đang deploy có nhánh đọc receipt mới nhất, dễ để ACK STOP che lỗi của lệnh đi trước. Đang sửa source HTML local để giữ lỗi motion và đối chiếu command ID.
- Lượt kiểm tra này thấy phiên quét đã SAVED, session_open=false và stack đã dừng. Không gửi lệnh bánh từ phiên kiểm tra.
- Người dùng đã chọn khôi phục cấu hình đã chạy trước đây. Đã bật lại ba cờ `commissioned`, `cart_footprint_validated`, `watchdog_validated`; giữ nguyên tốc độ, lidar/TF/pose guards, servo và camera. Metadata ghi rõ chưa tạo chứng nhận stop-test mới. Không phát lệnh bánh khác 0 trong lượt cài.
- Đã khởi động stack: một supervisor, một worker và một watchdog; HELLO được BE chấp nhận. IMU READY, safety gate NO_LEASE khi chưa có lệnh, vận tốc an toàn toàn bộ bằng 0. Đo 79 bản tin gate: 20,01 Hz, gap lớn nhất 55,6 ms. Camera/lidar mỗi nguồn một publisher.
- Watchdog vốn đã chạy trước khi khôi phục cờ; cờ này không thêm tiến trình hay thời gian chờ. Gate/watchdog kiểm local 20 Hz, không đợi ACK BE theo mỗi chu kỳ. Ngưỡng watchdog mất dữ liệu là 0,5 s; command 0,3 s và lease 0,4 s được giữ nguyên.
- `media.log` xác nhận phiên camera là MONITOR, không phải ASSIST/SOS giữ bánh.
- Source `BE_MAR/src/main/resources/static/mapping.html` đã sửa theo dõi receipt theo command ID và giữ lỗi di chuyển qua ACK STOP. Đã kiểm cú pháp và 4 tình huống trong RAM; chưa push/deploy. Pi có thể nhận điều khiển HTML hiện tại sau khi tạo job quét mới; job cũ đã SAVED.
- Bản sao cấu hình private đã đồng bộ sau áp dụng, candidate tạm trên laptop đã xóa. Không thêm file test lên Pi.

## 13. Đối chiếu chạy thẳng và định vị với source hãng cũ

- Đã kiểm toàn bộ 149 file production trên Pi: không thiếu file hoặc sai SHA so manifest. AMCL executable, Smac, RPP và native BT plugin đều có.
- Log manual mapping cho thấy gate nhận và truyền nguyên lệnh `(x>0,y=0,z=0)`, sau đó có `LIDAR_BRAKING_STOP`; các lượt quay bị `TURN_ENVELOPE_BLOCKED`. Autonomy đã dừng với `ROBOT_FOOTPRINT_NOT_CLEAR_IN_MAP`, không phải exception AMCL.
- Worker hiện dùng chế độ mapping/SLAM. AMCL chỉ bật bởi launcher navigation trên map đã lưu; voice wake và QR staff cũng chờ runtime navigation. Không bật hai nguồn map→odom cùng lúc.
- Đã tìm snapshot hãng tại `Downloads/MAR_Hiwonder_Audit_2026_10_20/src/driver/controller`. `mecanum.py` và `ekf.yaml` giống bản trên image hiện tại; nhánh Mecanum của cmd_vel_callback vẫn cùng công thức và dấu motor. Driver odometry có thay đổi phần lấy machine_type và nhánh Acker, nhưng actual MACHINE_TYPE là MentorPi_Mecanum. MAR remap ngõ app/cmd_vel sang topic không sử dụng.
- Khác biệt calibration xác nhận được: snapshot cũ linear 0,93 / angular 1,01; image hiện tại 1,0 / 1,0. Chưa tự đổi lại. Các hệ số không đổi dấu motor hoặc biến lệnh đi thẳng thành đi ngang.
- Odometry hãng là tích phân lệnh, không phải phản hồi encoder độc lập. EKF dùng twist x/y/yaw cùng IMU yaw/rate. Vì vậy odom thẳng không đủ chứng minh chassis thực sự đi thẳng; cần đối chiếu IMU và motor/quan sát thực tế.
- Đã chạy listener thụ động bằng RAM, không tạo file hay phát lệnh bánh. Cửa sổ quan sát chưa thu được lệnh khác 0. Lượt kiểm tra cuối stack đã dừng; chưa kết luận được nguyên nhân cơ khí/chiều motor của hiện tượng người dùng báo đi ngang/quay.

