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
