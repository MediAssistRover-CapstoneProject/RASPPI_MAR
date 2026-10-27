# Quét các phòng có lối nối – 26/10/2026

## Kết quả kiểm tra phiên thực

Pi đang cấu hình `exploration_mode=reactive`, `frontier_controller=reactive`, trần 1.0 m/s, 0.8 rad/s. Bộ reactive chỉ chọn hướng trống hiện tại; nó không chọn phòng/vùng chưa quét. `nav2_assist` trước đây bỏ toàn bộ vận tốc lùi tự động. Khi không đủ vòng quay và không có khoảng trượt ngang, robot có thể đứng chờ trong phòng dù đã đi vào được.

Các lần dừng ghi nhận lúc 12:22:32, 12:26:14 và 12:27:32 ngày 26/10 (+7) là `CONTROL_LEASE_EXPIRED`, không phải `FRONT_OBSTACLE_TOO_CLOSE`. Khoảng hở phía trước được log là 0.58 / 0.38 / 0.25 m. Mất gia hạn và khó thoát phòng là hai vấn đề khác nhau.

BE đã lưu thành công hai job `4dee73ee-e0cc-48f9-8417-a565defe8efe` và `01852942-9f57-45b7-80c9-84ab35f3e33d`. Lần đọc DB lúc 12:31 (+7) có job quét `4b9d5821-55b5-43db-b62e-d90fc1738b42` RUNNING. Theo yêu cầu người dùng, không đóng job này, không sửa code/config hoặc khởi động thêm tiến trình trên Pi trong lượt này.

Trên map đã lưu 226×234 ô, 2.5 cm/ô và tọa độ từ ảnh `(2.24, 2.57, 135°)`, planner mới kiểm tra footprint thân hiện tại là clear và tìm 7 frontier. Đây là phép tính trên snapshot; không chứng minh đường đó đã được Nav2 và phần cứng đi qua.

## Luồng mới

1. SLAM + LiDAR tạo map và pose trong phiên hiện tại; không cần AMCL cho lượt đang quét.
2. Frontier planner ưu tiên biên giữa vùng free và vùng chưa biết có kết nối trên map.
3. Khi chưa còn frontier phù hợp, chọn điểm quan sát free mà robot chưa đi qua. Ghi nhớ đường robot thực đã đi; khoảng phủ được tính dọc vùng free, không tính xuyên tường. Màu trắng nhìn từ hành lang không tự xác nhận đã đi hết phòng.
4. Nav2 kiểm đường bằng costmap/footprint, DWB điều khiển Mecanum đi tới mục tiêu. Đi ngang giữ nguyên; thêm lùi tối đa 0.10 m/s nếu profile phần cứng đã cho phép và LiDAR phía sau đủ dữ liệu/khoảng hở. Không dùng lệnh lùi mù khi Nav2 thất bại.
5. Điểm quét không bắt buộc quay đúng hướng cuối vì LiDAR quét 360°. Riêng MOVE tới endpoint trên map đã lưu vẫn giữ kiểm tra hướng cuối.
6. Mục tiêu kẹt/không tiến triển được dừng và bỏ qua tạm có thời hạn để thử mục tiêu khác. Không báo hoàn tất chỉ vì một cửa bị chặn.
7. Khi không còn frontier đáng kể và các điểm free tiếp cận được đã được đi qua, xác nhận ổn định trước `COVERAGE_COMPLETE`. Hết thời gian 600 giây là `TIME_LIMIT`, không phải chứng minh đã quét hết.

## Lease và vận tốc

Preset phòng dùng trần tiến/ngang 0.25 m/s, quay tối đa 0.6 rad/s và giảm tốc theo khoảng hở. Trần phần cứng, tốc độ tay cầm, camera và mạng không đổi. Giữ biên dừng thân đang cấu hình 5 cm; không giảm thêm footprint hoặc khoảng dừng để ép đi qua cửa.

Khi lease hết, đầu ra zero và thu hồi lease motor ngay. Chỉ giữ trạng thái lượt chạy tối đa 2 giây để một RENEW mới, đúng phiên/run/options và chưa hết hạn có thể phục hồi. Không có lệnh mới thì kết thúc `CONTROL_LEASE_EXPIRED`. STOP/đổi phiên/lỗi cảm biến vẫn dừng theo cơ chế hiện có. Giữ tab điều khiển ở foreground vì hiện UI vẫn chịu trách nhiệm gia hạn; bản này không biến mapping thành nhiệm vụ chạy độc lập hoàn toàn khi đóng trình duyệt.

## File thay đổi

- `pi/mediassist/frontier_mapping.py`: frontier và bộ nhớ các vùng đã đi qua.
- `pi/mediassist/autonomous_mapping.py`: lùi có giới hạn, chờ gia hạn nhưng không phát vận tốc.
- `pi/mediassist/mapping.py`: giữ zero trong khoảng chờ lease, chọn một nguồn điều khiển.
- `pi/mediassist/mapping_navigator.py`: dừng/loại mục tiêu không tiến triển, không nhận vận tốc cũ khi replanning.
- `tools/mapping_nav2_parameters.py`: đồng bộ reverse ở DWB/smoother, bỏ yêu cầu quay cuối chỉ trong mapping.
- `pi/mediassist/navigation_setup.py` và `pi/config/navigate_no_recovery.xml`: giữ footprint/unknown guard, dựng lại costmap và thử planner/controller có giới hạn.
- `tools/configure_room_exploration.py`: preset được áp dụng khi stack đã dừng, có backup, không gửi lệnh chuyển động.

Không cần thêm API hoặc đổi MQTT schema ở BE. Các reason mới dùng trường reason hiện có. BE/UI vẫn nhận số frontier, mục tiêu và trạng thái quét như trước.

## Giới hạn cần kiểm thử trên sàn

Room coverage ở đây là các điểm free có đường tới trên grid, không phải bộ nhận dạng tên phòng hoặc chứng nhận quét mọi ngóc ngách vật lý. Phòng đóng cửa, vùng không quan sát được, ô map sai, cửa không đủ footprint, đồ vật/ người mới đi vào hoặc SLAM trôi có thể khiến một vùng không đi tới được. Không xóa ô đen/xám để coi là đã quét. Thử từng cửa, vào phòng cụt và thoát lùi với người giám sát trước khi chạy toàn tầng.

Preset này dành cho thân Mecanum đã tháo xe kéo có khớp như đã xác nhận. Gắn xe kéo lại cần mô hình và kiểm thử khác; không dùng preset bare-body cho xe kéo.
