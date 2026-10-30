# Cấu hình MentorPi / MediAssistRover — 30/10/2026

Bản ghi lấy từ cấu hình thật trên Pi và workflow trong DB BE, không chứa password, token hoặc device secret. File JSON đi kèm là snapshot cấu hình để đối chiếu; không chép đè trực tiếp nó vào mqtt.worker.json.

## 1. Thiết bị và địa chỉ

| Mục | Giá trị đang dùng |
|---|---|
| Robot ID | `0e3f1083-2546-4f45-8004-5eda5cb75a6f` |
| Pi SSH | `pi@192.168.1.56` |
| BE HTTPS | `https://192.168.1.55` |
| Trang thử trên máy BE | `http://127.0.0.1:8080/mapping.html#mapNavigation` |
| Container | `MentorPi`, chạy ứng dụng bằng user `ubuntu` |
| Project trong container | `/home/ubuntu/mediassist-mqtt-2026-10-08` |
| Worker config | `pi/config/mqtt.worker.json` |
| Hardware profile hiện tại | `pi/config/site.motion.20261030T121949479168.json` |
| Chassis | MentorPi M1 mecanum, cấu hình hiện tại dành cho robot đã tháo xe kéo |
| Footprint dùng trong code | trước 0,12 m; sau 0,12 m; trái 0,10 m; phải 0,10 m từ base_footprint |
| ROS | ROS 2 Humble; `map` → `odom` → `base_footprint` |
| LiDAR | MS200; `/scan_raw`; normalizer 360 bins. 360 là số hướng mẫu, không phải Hz. FPS/Hz driver thực tế cần đo khi stack chạy. |

## 2. Tốc độ và khoảng cách

| Chế độ | Tiến | Lùi | Ngang | Quay |
|---|---:|---:|---:|---:|
| Tay cầm USB | 0,25 m/s | 0,25 m/s | 0,25 m/s | 0,8 rad/s |
| Tự quét map / frontier Nav2 | tối đa 0,8 m/s | tối đa 0,8 m/s | tối đa 0,8 m/s | tối đa 0,8 rad/s |
| MOVE trên map đã lưu | theo max_speed_mps của step, không quá 0,8 m/s | cho phép theo path và cảm biến, không quá trần step | **0 m/s** | tối đa 0,8 rad/s |
| SPEED zone | giảm trần tiến/lùi theo speedLimitMps của zone | cùng giới hạn zone | MOVE không đi ngang | giữ trần quay; SPEED hiện giới hạn vận tốc tịnh tiến |

Tốc độ tịnh tiến là độ lớn vector, không cộng đồng thời hai trục để vượt trần. Hardware profile vẫn có max_speed_mps=1,0, nhưng trần MOVE/auto đang dùng là 0,8. Một trần tốc độ không phải cam kết robot luôn chạy ở vận tốc đó.

- Khoảng đệm LiDAR danh nghĩa từ mép thân: 0,03 m. Rear guard: 0,16 m từ tâm, tương đương khoảng 0,04 m sau thân đang cấu hình.
- Ngưỡng gần cảm biến LiDAR: 0,05 m từ LiDAR; khác với khoảng cách tính từ mép thân.
- Quay tại chỗ kiểm tra đường bao cả thân. Có khoảng trống phía trước chưa có nghĩa đủ chỗ quay.
- Giảm tốc theo khoảng phanh với giả định giảm tốc 0,6 m/s² và độ trễ scan + 0,35 giây. Ở 0,8 m/s, riêng quãng phanh lý tưởng đã khoảng 0,53 m, chưa tính độ trễ. Không đợi tới 3 cm mới phanh.
- DWB/velocity smoother: gia tốc thẳng 0,4 m/s², giảm tốc 0,6 m/s²; gia tốc quay 1,2 rad/s², giảm tốc quay 1,8 rad/s².
- Tay cầm: gia tốc thẳng 0,8 m/s²; gia tốc quay 2,5 rad/s²; mất mẫu joy quá 0,25 giây sẽ dừng.
- Cấu hình khoảng đệm 3 cm chưa được đo xác nhận trên sàn; bản đồ 2,5 cm/ô và sai số cảm biến/định vị không cho phép khẳng định dừng chính xác 3 cm.

## 3. StepType và giới hạn

| StepType | Thực thi | Quy tắc chuyển động |
|---|---|---|
| MOVE | Pi / Nav2 | max_speed_mps > 0 và ≤ 0,8; lấy mức thấp nhất của step, profile, zone và bộ phanh. Không đi ngang. |
| EXECUTE_SENSING / MAPPING | Pi, điều phối bởi BE | START/SAVE/STOP là vòng đời map; tự chạy là lệnh mapping-autonomy riêng, dùng trần auto 0,8 m/s. |
| EXECUTE_SENSING / ROBOT_HEALTH, SIDE_CLEARANCE, CROWD_DENSITY, RESTRICTED_ZONE | Pi hoặc producer AI | Bản thân sensing không ra lệnh di chuyển. AI detection RESTRICTED_ZONE khác với vùng cấm dẫn đường KEEP_OUT. |
| CONTROL_HARDWARE | Pi | Điều khiển servo khoang; không dùng tốc độ m/s. MG996R S6/ROS ID 4; khóa 1500, mở 2400, duration 1 giây; chưa có phản hồi vật lý vị trí chốt. |
| INTERACT | BE + Pi cho phát âm thanh | Không ra lệnh chassis. Speech có speed_wpm 80–220; đây là tốc độ giọng đọc. |
| WAIT | BE/Pi theo loại chờ | Không tự phát lệnh chassis; WAIT/DURATION 1–3600 giây. Tự quét đã bật có vòng điều khiển riêng trong phiên mapping. |
| DATA_SERVICE, AUTHENTICATE, HANDLE_PAYLOAD, NOTIFY, CHECK_CONDITION, SET_CONTEXT | BE theo schema hiện tại | Không có vận tốc chassis; workflow phải có MOVE riêng nếu cần đi. |

Pi hiện bật: MOVE, EXECUTE_SENSING, WAIT, CONTROL_HARDWARE, INTERACT.

Các workflow MOVE đã PUBLISHED trong DB tại thời điểm ghi:

| Workflow | Phiên bản | Trần tốc độ | Timeout step |
|---|---:|---:|---:|
| WF_MAP_ENDPOINT_MOVE | 1 | 0,05 m/s | 180 s |
| WF_MAP_ENDPOINT_MOVE | 2 | 0,15 m/s | 180 s |
| WF_MAP_ENDPOINT_MOVE | 3 | 0,8 m/s | 180 s |
| WF_ENDPOINT_MOVE_CHECK | 1 | 0,05 m/s | 90 s |
| WF_NAVIGATION_SINGLE_ENDPOINT_MOVE | 1 | 0,05 m/s | 180 s |

Nút Chuẩn bị workflow MOVE trên HTML đối chiếu contract rồi chọn/tạo phiên bản có max_speed_mps=0,8. Các phiên bản cũ giữ giới hạn riêng. Timeout là thời gian tối đa thực thi, không phải chu kỳ kết nối.

## 4. Zone đã chốt và API

| Tên trên giao diện | Mã API | Thực thi |
|---|---|---|
| Speed limit zone | SPEED | Giảm trần tiến/lùi khi tiếp cận và trong vùng; nhiều vùng chồng nhau lấy giới hạn thấp nhất. |
| Restricted zone | KEEP_OUT | Vẽ mask vào local/global costmap Nav2; planner tìm đường tránh. Có thêm chặn vận tốc khi gần biên vùng. Không coi vùng cấm là nơi được phép đi vào rồi mới xử lý. |

API đã có:

- `GET /api/v1/zones?page=0&size=100`
- `POST /api/v1/zones`
- `PUT /api/v1/zones/{zoneId}`
- `POST /api/v1/zones/{zoneId}/activate` và `/deactivate`
- `GET /api/v1/robots/{robotId}/navigation-zones` — ADMIN, cho biết profile hiện tại đã được Pi ACK hay chưa.
- `GET /api/v1/pi/maps/{mapId}/zones/config` — DEVICE, danh mục zone của map được phép đọc.

Ví dụ body để chỉnh theo vị trí thật; không phải zone đã tạo sẵn:

```json
{
  "mapId": "UUID_MAP_DA_LUU",
  "zoneCode": "SLOW_01",
  "zoneName": "Khu giảm tốc trước cửa",
  "zoneType": "SPEED",
  "polygonJson": {"points": [[0,0], [1,0], [1,1], [0,1]]},
  "speedLimitMps": 0.15,
  "active": true,
  "publicVisible": false
}
```

Restricted zone dùng zoneType=KEEP_OUT và speedLimitMps=null. Polygon phải đơn, không tự cắt, có diện tích khác 0, gồm 3–256 điểm. Tất cả tọa độ là mét trong frame map; xoay hình hiển thị không đổi tọa độ lưu.

Luồng: API/DB zone → NAV_PROFILE chứa polygon/rule/hash → Pi kiểm tra map/frame và phát mask → ACK → BE cho dispatch → MOVE/Nav2 và bộ giới hạn vận tốc thực thi rule. UI hiển thị xác nhận profile, không thay thế kiểm chứng chuyển động thực tế.

Chỉnh zone khi robot rảnh; rule mới áp dụng sau đồng bộ. MOVE đang chạy giữ profile đã chấp nhận. Zone map đã lưu áp dụng trong navigation của đúng map đó, không tự áp dụng lên phiên SLAM mới có hệ tọa độ khác. Không tạo zone ở tọa độ đoán để thử với robot thật.

## 5. Endpoint đã chốt

| Loại | Mã API | Ý nghĩa |
|---|---|---|
| Parking endpoint | PARK | Chỗ đỗ thường xuyên, điểm xuất phát quen thuộc; có thể chọn làm ước lượng khởi tạo AMCL khi xác nhận robot thật ở đúng đây. |
| Waypoint endpoint | WAYPOINT | Điểm đến dẫn đường/giao đồ hoặc điểm trung gian theo workflow. |
| Manual Charger endpoint | CHARGER | Robot đi tới vị trí sạc; người vận hành cắm sạc. Đến endpoint không đồng nghĩa đang sạc hoặc pin đầy. |

API: GET/POST `/api/v1/endpoints`, PUT `/api/v1/endpoints/{endpointId}`. Các loại STAGING/NO_STOP cũ được giữ để tương thích dữ liệu, không thêm vào lựa chọn chính mới.

Endpoint lưu xM, yM, yawRad và mapId; zoneId là liên kết danh mục. Việc thực thi zone căn cứ toàn bộ polygon đang bật, không chỉ zoneId gắn với endpoint. Có thể MOVE về PARK hoặc CHARGER bằng cùng workflow MOVE. Không tự phát sinh hành vi dock/charging/khóa khoang từ tên loại endpoint.

## 6. Cách thử trên HTML

1. Mở mapping.html, dùng token ADMIN, chọn Robot ID và Map ID, tải map. Kết thúc job quét trước khi navigation.
2. Kích hoạt đúng map, mở một phiên navigation và chạy lệnh Pi mà trang cung cấp. Một phiên dùng cho nhiều job A→B; không cần khởi động phiên mới cho từng endpoint.
3. Tạo PARK ở chỗ đỗ thật, WAYPOINT ở đích, CHARGER ở chỗ sạc. Bấm/kéo chọn vị trí và hướng rồi lưu. Nếu robot thật đang đỗ ở PARK, chọn endpoint đó, xác nhận checkbox và bấm Dùng parking làm vị trí khởi tạo; sau đó Gửi vị trí khởi tạo cho AMCL. Chờ READY.
4. Trong Zone trên map, bấm Zone mới, chọn rule, bấm từng đỉnh trên map; có thể sửa JSON điểm. SPEED cần tốc độ, ví dụ 0,15 m/s. Lưu zone và chờ Pi xác nhận profile mới.
5. Chuẩn bị workflow MOVE, chọn waypoint và gửi. Đường đi phải tránh vùng đỏ và vận tốc giảm khi tiếp cận vùng vàng. Hủy dẫn đường nếu cần dừng giữa chừng, chờ Pi xác nhận dừng trước job tiếp.

AMCL vẫn phải theo dõi pose cùng odometry/TF khi chạy. Parking chỉ là ước lượng vị trí ban đầu, không cho phép dùng mãi tọa độ cũ sau khi robot đã di chuyển.

## 7. Camera và vận hành

Camera dùng ascamera + web_video_server của hãng, cổng 8080, 640×480, target 15 FPS, quality 60. MQTT preview đang tắt. Driver HP60C đã từ chối thử 20 FPS; 15 FPS là cấu hình được chấp nhận, không phải số FPS thực tế đã đo trong mọi tải.

Camera: `http://192.168.1.56:8080/stream?topic=/ascamera/camera_publisher/rgb0/image&type=mjpeg&width=640&height=480&quality=60`

Heartbeat worker 3 giây; BE timeout config 15 giây; lease 0,4 giây; command timeout 0,3 giây; scan braking tối đa 0,4 giây. Những ngưỡng này khác với thời hạn phiên navigation. Chúng phát hiện mất lệnh/cảm biến; không chủ động khởi tạo lại vị trí ở mỗi heartbeat.

Dừng Ctrl+C: dừng và xác nhận vật lý, ghi FAILED/PROCESS_SHUTDOWN cho bước đang chạy khi có bằng chứng, chờ gửi kết quả về BE. Giữ journal chống trùng. Mất điện/thiếu bằng chứng dừng vẫn cần recovery; không tự xóa SQLite.

## 8. Trạng thái kiểm tra

BE và Pi đang dùng địa chỉ ở mục 1; HTTPS/device auth và TLS broker đã kiểm tra. BE build được, kiểm thử API/contract/geometry và UI tự động đạt. Pi đã có module zone, mask Nav2 và MOVE không đi ngang; đã thử tạo mask bằng ROS khi không chạy bánh.

Ở lần kiểm tra này Pi đang dừng stack, robot OFFLINE trên BE, không có currentMap hoặc phiên navigation đang mở. Vì thế profile navigation cần chờ người vận hành chọn đúng map; đây không phải lỗi IP/CA. DB hiện có một SPEED zone đang bật tên hehhee, map eea6c2b7-46e8-4039-b184-5870aea5a341, tốc độ 0,22 m/s. Các endpoint hiện có đều là WAYPOINT; chưa có PARK hoặc CHARGER được đặt theo vị trí thật. Không tạo sẵn polygon hoặc parking ở vị trí đoán.

Chưa thử robot thật đi qua zone sau cập nhật UI/API. Không thể cam kết không va chạm, định vị tuyệt đối hoặc phù hợp triển khai bệnh viện chỉ từ kiểm thử phần mềm. Kiểm thử timing tái định vị AMCL cũ đã có một trường hợp chưa đạt; phần zone không được coi là bản sửa hoàn tất lỗi đó.
