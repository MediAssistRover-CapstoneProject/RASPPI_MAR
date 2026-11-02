# MOVE, đường cong và giữ định vị — 2026-11-02

Trong một phiên navigation, AMCL khởi tạo theo revision của phiên; tạo job MOVE tiếp theo không tự tạo revision hoặc gửi lại initial pose. Sau khởi tạo, AMCL vẫn hiệu chỉnh map→odom từ LiDAR, encoder và IMU. Không đóng băng tọa độ ban đầu: robot đi thì vị trí phải thay đổi. Mất chất lượng định vị khác với mất MQTT.

Giữa đường, RPP phát vận tốc tiến và quay đồng thời ở 20 Hz để bám đường Smac/Nav2. Đây là điều khiển liên tục, không phải BE gửi một MOVE mới cho mỗi điểm path. Các góc cua vừa được đi theo cung; góc lệch lớn hơn 0.75 rad (~43 độ) có thể cần quay tại chỗ. Nav2 có thể cập nhật path khi có vật cản; đường hiển thị là kế hoạch mới nhất được Pi/BE xác nhận.

Khi vào zone giảm tốc, Pi giảm vận tốc tiến và vận tốc quay theo cùng tỉ lệ để giữ độ cong của path. Vận tốc sau ramp cũng chịu giới hạn zone; restricted zone vẫn chặn chuyển động. Thoát speed zone thì trần trở về giới hạn MOVE. Không bỏ kiểm tra va chạm để ép robot đi hết path.

Mọi endpoint, kể cả điểm trung gian, vẫn dùng hướng đã lưu: tới vị trí, dừng để xác nhận, gửi tối đa một action Spin nếu cần, rồi xác nhận vị trí/hướng và kết thúc step. Nếu quay bị vật cản/zone chặn thì step thất bại có bằng chứng dừng; không báo thành công sai hướng. Số lệnh vận tốc trong action Spin không phải nhiều action quay độc lập.

| Thông số | Giá trị |
|---|---|
| MOVE tối đa trên Pi | 0.60 m/s; không đi ngang/lùi |
| Tự quét tối đa | 0.80 m/s; holonomic, cấu hình quét giữ nguyên |
| Quay tối đa | 1.20 rad/s, còn chịu gate/zone |
| Ngưỡng quay tại chỗ khi bám path | 0.75 rad |
| Dung sai goal checker XY | 0.06 m |
| Xác nhận kết thúc vị trí / hướng | ≤0.08 m / ≤0.10 rad (~5.7°) |
| Chờ ổn định vị trí cuối | tối đa 10 s |
| Giữ giả thuyết định vị khi đứng yên có odom mới | tối đa 60 s; phải có scan mới đạt chuẩn mới READY |
| Giữ qua lỗi HTTPS tạm thời | tối đa 5 s kể từ descriptor tốt cuối; không gia hạn do lỗi |
| Heartbeat Pi | 3 s |
| Timeout ACK nghiệp vụ Pi | 15 s |
| Gửi lại event Pi / command BE | 8 s, cùng identity, chống thực thi trùng |
| TTL command BE | 30 s |
| Timeout mỗi MOVE | 180 s |

Đo ngày 02/11 trên mạng thử: SELECT 1 trên kết nối Aiven đã mở 39–74 ms; mở kết nối TLS/JDBC mới khoảng 1.1 s. COMMAND_ACK/STEP_STARTED từ timestamp Pi tới BE commit có mẫu 4.8–6.5 s. Khoảng này gồm mạng, hàng đợi và xử lý transaction, không thể quy hết cho một truy vấn DB. BE ghi thêm queueMs, storeMs, processMs và ackScheduleMs để phân biệt. Gửi lại sau 8 s tránh replay trong cửa sổ commit thông thường, không làm nhanh thời gian xử lý gốc.

Log tuyến fc04aac4…: endpoint A thành công; endpoint B tới vị trí nhưng Spin cuối khoảng -85° bị Nav2 báo Collision Ahead, nên FINAL_ALIGNMENT_BLOCKED. Không có MQTT disconnect trong lượt này. Tuyến sau vẫn tạo được, nhưng gặp ENDPOINT_DRIFT_DURING_ALIGNMENT. Không bỏ hướng endpoint để che hai lỗi này.

Cài khi stack đã dừng. Phiên mới sinh lại YAML Nav2 từ navigation_setup.py. Không chạy robot.zsh run đồng thời navigation_session.zsh. Các thay đổi không chỉnh IP, CA/TLS, secret, map, endpoint hoặc SQLite journal. Cần chạy thực tế trên sàn để đánh giá đường cua, các cửa hẹp và hướng cuối; kiểm thử phần mềm không chứng minh sai số vật lý bằng 0.

Sửa bổ sung khởi tạo Nav2: hai job `145be3c1…` và `113f3466…` FAILED/PLANNER_UNAVAILABLE dù AMCL đã LOCALIZED. Log controller_server cấu hình RPP thành công nhưng phản hồi `/controller_server/change_state` bị timeout; controller INACTIVE, planner và các node sau vẫn UNCONFIGURED. Đây là lỗi lifecycle cục bộ, không phải lỗi tải map Aiven hoặc MQTT.

`nav2_startup.py` cấu hình từng node bằng RPC có timeout và đọc lại GetState; nếu phản hồi mất nhưng node đã INACTIVE thì không gửi CONFIGURE trùng. Sau đó dùng RESUME của lifecycle manager Nav2 gốc để kích hoạt và duy trì bond. Không thay quản lý bond hoặc kiểm tra va chạm của Nav2. Cấu hình mỗi node giới hạn 15 giây; activation tối đa 180 giây để chờ TF/initial pose. Nếu không hoàn tất thì launch dừng rõ lỗi, không treo startup vô hạn. Worker chỉ gọi action sau khi cả sáu node ACTIVE và manager xác nhận hoạt động; chờ tối đa 15 giây rồi báo lỗi kèm trạng thái thực tế. Không gửi goal hoặc vận tốc từ bộ khởi tạo.

Sau khi cài, mở phiên navigation mới trên UI rồi chạy dòng lệnh UI cung cấp. Kiểm tra `NAV2_ACTIVE_WITH_VENDOR_BONDS` trong slam.log, sau đó AMCL LOCALIZED trước khi tạo MOVE. Mã tham khảo: [Nav2 Humble lifecycle manager](https://github.com/ros-navigation/navigation2/blob/humble/nav2_lifecycle_manager/src/lifecycle_manager.cpp) và [lifecycle service client](https://github.com/ros-navigation/navigation2/blob/humble/nav2_util/src/lifecycle_service_client.cpp).

Bổ sung nhóm localization: lần chạy 08:24 UTC đọc map.pgm 231×188 ô thành công, nhưng phản hồi CONFIGURE của map_server bị mất, nên không ACTIVATE và chưa phát /map. Bộ khởi tạo có timeout/đọc lại trạng thái nay áp dụng riêng cho map_server/AMCL qua lifecycle_manager_localization; khởi động nhóm dẫn đường chậm hơn nhóm localization 3 giây để giảm dồn DDS discovery. Nav2 dùng PGM/YAML, không dùng PNG để định vị. Đã thử bằng ROS Humble thật trên Pi ở domain 73 tách biệt: map_server và AMCL ACTIVE qua manager gốc, nhận /map đúng Map ID e571fe71-d3ea-4aa5-a673-ffd857ac9f09, 231×188, resolution 0.02500000037252903, frame map; helper thoát 0. Không chạy chassis, không gửi goal/velocity; các tiến trình kiểm tra đã được dừng.

Ổn định runtime sau đó: request e6f4aaf9… FAILED/CHASSIS_COUNT_INVALID dù chỉ có một tiến trình odom_publisher. Graph ROS thiếu tên driver và báo `_NODE_NAME_UNKNOWN_` tại endpoint. Các wrapper dự án nay thống nhất `rmw_cyclonedds_cpp`, `ROS_DOMAIN_ID=42`, `ROS_LOCALHOST_ONLY=1`; không sửa image/vendor .zshrc. Cyclone đã có sẵn trên Pi, không cài thêm package. `pi/config/cyclonedds.xml` cấp chỉ số participant tự động tới 120 và peer loopback; không kế thừa biến CYCLONEDDS_URI của image đang trỏ tới file không tồn tại. MQTT, camera HTTP, BE/Aiven và CA không phụ thuộc ROS domain.

Trong terminal chẩn đoán phải dùng cùng môi trường, vì source .zshrc của hãng hiển thị domain 0:

```sh
cd /home/ubuntu/mediassist-mqtt-2026-10-08
source tools/ros_environment.zsh
ros2 node list --no-daemon
```

Không chạy hai stack khác middleware/domain cùng điều khiển phần cứng. Wrapper vẫn kiểm tra/dừng bringup hãng trùng như trước. Topology audit giữ nguyên điều kiện duy nhất một chassis và đường motor qua gate/watchdog; kiểm tra đầy đủ trước mỗi MOVE và tối đa 2 Hz trong runtime, không quét graph ở mỗi vòng điều khiển 20 Hz. Lỗi graph nay ghi rõ danh sách driver/subscriber.

Scan match giảm ngắn sau khi đã khóa vị trí: grace tối đa 2 giây và cap MOVE 0,15 m/s, giữ độ cong; phục hồi thì trở về giới hạn workflow/zone/Pi. Không thay ngưỡng bootstrap, không tự reset AMCL hay gửi initial pose theo từng job. Scan rất thấp, covariance sai, nhảy pose, TF/cảm biến cũ vẫn chặn ngay; giảm chất lượng kéo dài vẫn giữ/dừng goal. Lượt 08:38 UTC dừng do raw scan match từ 0,525 xuống 0,457 rồi kéo dài, không phải DB/ngắt MQTT. Không thể suy ra bánh xe chạy đúng chỉ từ vận tốc odometry.

Kiểm thử sau thay đổi: 76 ca phần mềm liên quan qua; ROS Humble/Cyclone thật ở domain 73 nạp đúng map, kích hoạt map_server/AMCL và đủ Nav2, kiểm tra được custom MotorsState type support, ComputePathToPose trả SUCCEEDED với 17 pose. Dữ liệu scan/odom/TF dùng fixture trong domain tách biệt; không có chassis, không gửi NavigateToPose hoặc lệnh motor. Đây là kiểm chứng khởi tạo/lập đường, chưa phải xác nhận độ mượt bánh xe thực tế. Tất cả tiến trình fixture đã dừng.
