# Đối chiếu Hiwonder và MAR — 23/10/2026

## Phạm vi và bằng chứng

Đã đọc source Hiwonder người dùng cung cấp: `Downloads/Source Code-20260820T100410Z-1-001/Source Code/src.zip`, repo `RASPPI_MAR/PI_MAR`, và source BE đang chạy trong `Downloads/MAR_EMQX_Demo/BE_MAR`. Đây là kiểm tra source, không phải xác nhận mọi tham số hiện đang chạy trên Pi. Người dùng đã thử trực tiếp và xác nhận video server hãng ở 8080 mượt hơn nhánh Python 8081 trên cùng robot.

## Các thành phần dùng lại được

### Đối chiếu thêm footprint, initial pose và tự quét

Đã đọc lại trực tiếp các file trong ZIP ngày 23/10: `navigation/config/nav2_params.yaml`, `navigation/config/nav2_controller_dwb.yaml`, `peripherals/launch/include/ms200_scan.launch.py`, `peripherals/launch/lidar.launch.py`, `app/app/lidar_controller.py`, `simulations/mentorpi_description/urdf/lidar.urdf.xacro`, `mecanum.xacro`, `peripherals/peripherals/joystick_control.py` và các launch SLAM/navigation.

- MS200 dùng `oradar_lidar/oradar_scan`, serial 230400, range 0,05..12 m, góc 0..360°, `clockwise=False`, `motor_speed=15` trong ZIP. Đây là giá trị cấu hình nguồn; không suy ra tần số đo thực tế bằng 15 Hz nếu chưa đo runtime.
- `lidar_frame` trong URDF kèm ZIP xoay yaw π so với `lidar_link`; không coi chỉ số đầu mảng scan là hướng trước của `base_footprint`. Runtime TF mới quyết định hướng thực tế. URDF nằm dưới package simulations; cần đối chiếu URDF đã cài trên Pi trước khi coi là hình học phần cứng chính xác.
- `peripherals/launch/lidar.launch.py` có khai báo laser filter nhưng dòng thêm `laser_filter_node` vào launch đang comment. Không thể kết luận driver đang tự lọc điểm vào thân robot. Không bật filter bỏ tia gần 10 cm để che lỗi va chạm; trước hết kiểm tra raw scan/TF/đồ gắn.
- `nav2_params.yaml:41` bật `set_initial_pose: True`, x/y/yaw bằng 0 ở các dòng 42..45. Đây là vị trí giả định, không phải tự nhận ra vị trí trên map.
- Cùng YAML đặt `robot_radius: 0.05` ở local/global costmap. Bán kính này không bao thân robot 21,2 × 17,1 cm người dùng cung cấp. MAR dùng polygon thân riêng khi tạo Nav2 YAML.
- AMCL hãng chọn `DifferentialMotionModel`; công cụ localization MAR chọn `OmniMotionModel` cho Mecanum có dịch ngang. Giữ model phù hợp động học thực tế.
- Joystick hãng ánh xạ `lx → linear.y`, `ly → linear.x`, `rx → angular.z` ở dòng 66..69; preset `vendor_mecanum` MAR dùng cùng ý nghĩa trục. Không bật thêm node hãng phát vận tốc trực tiếp song song.
- Tránh vật cản hãng mặc định threshold 0,6 m, speed 0,2 m/s; service chỉ nhận threshold 0,3..1,5 m. Do đó ngưỡng dừng 0,10 m người dùng yêu cầu không phải preset mặc định của hãng. MAR phân biệt hard stop quanh LiDAR với giảm tốc/tránh và kiểm tra thân.
- Không thấy frontier/coverage completion trong `app/app/lidar_controller.py`; node này không thay phần xác nhận quét xong/lưu map/job của MAR.

Xem `LOCALIZATION_GEOMETRY.md` cho bản sửa hình học, chẩn đoán và luồng định vị map tĩnh. Chưa chứng minh hiệu năng phần cứng chỉ bằng đọc source hãng; tối ưu tiếp cần đo scan/odom/SLAM/CPU trên đúng runtime.

| Phần | Source Hiwonder | MAR hiện tại |
|---|---|---|
| Camera RGB/depth | `peripherals/launch/include/ascamera.launch.py`, `ascamera_node` | Cùng driver qua launch phần cứng hãng; LAN trước đây dùng Python, bản vá mới cho chọn `web_video_server` đã cài trên Pi |
| LiDAR MS200 | `peripherals/launch/include/ms200_scan.launch.py`, package `oradar_lidar`, executable `oradar_scan` | Dùng driver hãng qua `slam/launch/include/robot.launch.py`, không tự viết bộ đọc UART LiDAR |
| Odometry, IMU, EKF | `driver/controller/launch/controller.launch.py`, controller, IMU filter, `ekf_node` | Dùng launch phần cứng hãng; giữ remap lệnh vận tốc qua gate của MAR |
| Dựng map 2D | `slam/launch/include/slam_base.launch.py` chạy `slam_toolbox/sync_slam_toolbox_node` | Cùng engine; `pi/mediassist/slam_session.py` đọc YAML hãng, điều chỉnh một số tham số và quản lý graph riêng từng phiên |
| Định vị map cũ | `navigation/launch/include/localization.launch.py` dùng `nav2_map_server` và `nav2_amcl` | Cùng công nghệ; thêm kiểm tra map/hash, lifecycle, TF và mức tin cậy định vị |
| Dẫn đường | Nav2, DWB/NavFn trong cấu hình ZIP | Nav2 chuẩn Humble + cấu hình dựa trên YAML hãng, footprint thực, giới hạn tốc độ, endpoint/yaw và adapter workflow |
| Lưu map | `slam/slam/map_save.py` gọi `nav2_map_server map_saver_cli`, tên cố định `map_01` | Lưu snapshot OccupancyGrid thật thành PGM/YAML, gắn session/artifact/hash, upload HTTPS rồi BE đăng ký map |
| Tránh vật cản | `app/app/lidar_controller.py`: tránh vật cản, bám theo, canh gác | Có thể dùng thuật toán làm tham khảo; không chạy thẳng node hãng song song vì phát trực tiếp `/controller/cmd_vel` |
| AI/thị giác | App tracking, hand gesture, line following, AR… | Có thể bọc adapter theo StepType khi cần; mỗi node phải xác định camera consumer, tài nguyên và quyền phát lệnh động cơ |

## LiDAR/map có dùng web server giống camera không?

Không dùng cùng loại HTTP video server. LiDAR phát `sensor_msgs/LaserScan` qua ROS 2; SLAM lấy scan + odom/TF để tạo `nav_msgs/OccupancyGrid` trên `/map` và TF `map → odom`. MAR gửi preview/pose/metadata lên BE qua MQTT. Web hiển thị dữ liệu BE, không mở MJPEG để đọc LiDAR.

`nav2_map_server` cũng là ROS node/service quản lý map tĩnh, không phải HTTP server để trình duyệt gọi trực tiếp. Source hãng còn khởi động `rosbridge_server`; không có source app WonderPi trong ZIP nên không khẳng định toàn bộ giao thức phía app chỉ từ các launch file.

Nếu bật `scan_normalization`, MAR thêm `/mediassist/scan_slam` với 360 ô góc cố định trước SLAM để xử lý log số tia thay đổi 445/447/449. Driver MS200 và scan thô cho kiểm tra chuyển động vẫn giữ nguyên. Chỉ kích hoạt theo cấu hình; không suy ra đang bật nếu chưa đọc cấu hình Pi.

## Các khác biệt cần giữ hoặc thử có kiểm soát

Thông số dưới đây là **YAML trong ZIP** và **mặc định loader MAR**, chưa phải phép đọc runtime Pi:

| Tham số SLAM | Hãng | MAR |
|---|---:|---:|
| `resolution` | 0,05 m/ô | 0,025 m/ô |
| `minimum_travel_distance` | 0,1 m | 0,05 m |
| `minimum_travel_heading` | 0,5 rad | 0,1 rad |
| `minimum_time_interval` | 0,5 s | 0,15 s |
| `map_update_interval` | 0,2 s | 0,5 s |

MAR dựng ô nhỏ hơn và nhận thêm chuyển động nhỏ; có thể tăng tải. Có thể thử preset hãng cho một phiên quét mới và đo tải/chất lượng map, nhưng không ghi đè map/endpoint cũ hoặc thay mọi tham số trong một lần. Tần số LiDAR, tốc độ cập nhật OccupancyGrid và tần số web preview là ba thông số khác nhau.

Các điểm không chép nguyên:

- YAML Nav2 trong ZIP đặt `robot_radius: 0.05`, không đại diện robot có xe kéo khớp nối.
- DWB trong ZIP đặt `max_vel_x: 0.4`, trong khi launch joystick mapping thực đặt `max_linear: 0.2`. Đây là các đường điều khiển khác nhau, không phải một tốc độ chung.
- Tránh vật cản hãng là phản xạ theo scan, không chứng minh quét phủ toàn phòng hoặc tự hoàn tất map. Không thấy thuật toán coverage/frontier trong `app/app/lidar_controller.py`.
- Script hãng lưu đè tên `map_01`; không thay thế được session/artifact/hash/receipt của MAR.
- Không chạy cả bringup/joystick/avoidance hãng cùng bộ điều khiển MAR mà chưa remap và phân quyền nguồn vận tốc.
- Xe kéo có khớp không được mô hình hóa đầy đủ chỉ bằng footprint chữ nhật. Cần kiểm chứng vùng quét khi rẽ và planner/giới hạn quay phù hợp; các bản kiểm thử offline chưa chứng minh được khả năng tránh va chạm cơ học của xe kéo.

## Giữ các luồng BE hiện có

Kiến trúc dùng hãng làm nền đã có: driver Hiwonder/ROS chuẩn → adapter MAR → MQTT/HTTPS → BE/HTML. Thay nhánh hiển thị camera không cần thay StepType, job ID, robot ID, map ID hoặc token.

Phần phải kiểm tra khi đổi cấu hình SLAM/Nav2: tên topic/frame, timestamp/QoS, chỉ một driver mỗi thiết bị, một nguồn vận tốc được chọn, map/hash/geometry đúng, yaw endpoint đúng, cancel/stop hoạt động, và pipeline lưu map hoàn tất. Không có đảm bảo tuyệt đối chỉ bằng việc dùng YAML hãng.

Luồng map: START_MAPPING → SLAM/preview → yêu cầu lưu → dừng chuyển động → SAVE_MAPPING (PGM/YAML + manifest) → upload HTTPS có DEVICE auth → COMMIT_MAPPING trên BE → STOP_MAPPING. Hủy/abort và lưu map là thao tác khác nhau; hủy không mặc nhiên cam kết lưu map.

BE hiện có `GET /api/v1/maps/{mapId}/image.png` và `GET /api/v1/maps/{mapId}/image-metadata`. Ảnh PNG lấy từ bộ YAML/PGM đã xác minh, không dựng ảnh giả từ kích thước/origin.

## Camera khi BE deploy lên cloud

Hai đường truyền hiện tại:

1. Pi chủ động kết nối MQTT broker và gọi HTTPS BE: job, heartbeat, preview map, pose, ảnh MQTT và upload map có thể qua Internet. Cần DNS, đồng hồ, TLS, cấu hình endpoint và credential hợp lệ.
2. Video MJPEG: trình duyệt chủ động kéo `http://IP_PI:8080/stream?...`. BE hiện không relay video này. `192.168.1.22` chỉ là IP LAN; Internet trên Pi không tự khiến URL LAN truy cập được từ xa.

Nếu người xem ở cùng LAN Pi, vẫn có thể mở URL Pi trực tiếp. Nhưng trang HTTPS cloud hiện từ chối URL camera HTTP, và trình duyệt có hạn chế mixed content. Cần endpoint camera HTTPS phù hợp hoặc xem trang video riêng trong mạng nội bộ.

Các cách đưa video ra xa:

- Demo: mạng VPN giữa Pi và máy xem, kèm HTTPS cho URL nhúng vào trang HTTPS.
- Tích hợp MJPEG: Pi mở đường hầm outbound đến gateway có xác thực; gateway/reverse proxy cung cấp HTTPS stream cho người dùng. Cần xử lý quyền truy cập robot, streaming không buffer, timeout và băng thông. Đây là phần cần bổ sung, chưa có trong bản vá camera.
- Sản phẩm nhiều người dùng: WebRTC gateway + signaling, STUN/TURN; Pi dùng encoder phù hợp. Giữ MQTT cho lệnh/telemetry. Đây là thiết kế tiếp theo, chưa được triển khai.

Mở port Internet là một khả năng triển khai riêng, còn phụ thuộc NAT/CGNAT/routing và xác thực/TLS. Không coi việc public thẳng server video demo là tính năng cloud đã hoàn tất. Native MJPEG không có nghĩa video đang được push qua MQTT hoặc BE.

## Bản vá camera đi kèm

`stream_backend=web_video_server`, cổng mặc định đã thử 8080, dùng executable từ package đã cài trên Pi; không chạy thêm driver/bringup. Python stream cũ vẫn có lựa chọn quay lại. Cấu hình cũ mặc định vẫn dùng Python đến khi chạy helper đổi backend. Không sửa YAML camera hoặc FPS20 đang được thử, MQTT, BE, job/map hoặc motor configuration.

Kiểm thử local: 48 ca camera/config/HTTP/supervisor đã qua. Video hãng được người dùng xác nhận mượt hơn; bản tích hợp auto-start mới chưa cài/kiểm thử thực tế trên Pi.

## Nguồn tham khảo chính

- Source ZIP Hiwonder do người dùng cung cấp, các file được chỉ rõ trong bảng.
- https://github.com/RobotWebTools/web_video_server
- https://github.com/SteveMacenski/slam_toolbox/tree/ros2
- https://github.com/ros-navigation/navigation2/tree/humble/nav2_amcl
- https://developer.mozilla.org/en-US/docs/Web/Security/Defenses/Mixed_content
- https://www.rfc-editor.org/info/rfc1918/
