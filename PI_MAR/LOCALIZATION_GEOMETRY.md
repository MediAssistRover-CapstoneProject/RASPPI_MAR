# Footprint, LiDAR 10 cm và định vị map — 23/10/2026

## Kết quả kiểm tra

- Source Hiwonder được đọc trực tiếp từ `Source Code-20260820T100410Z-1-001/Source Code/src.zip`.
- MAR dùng driver MS200 `oradar_scan`, bringup controller/odom/EKF hãng và `sync_slam_toolbox_node`; không có bộ đọc UART LiDAR riêng thay driver hãng.
- Camera native vẫn dùng `web_video_server` cổng 8080 đã được bạn xác nhận mượt. Bản này không đổi camera.
- Trong ZIP hãng, AMCL bật `set_initial_pose: True` với x/y/yaw bằng 0; Nav2 dùng `robot_radius: 0.05`. Không dùng hai mặc định đó để khẳng định robot đang ở đúng gốc map hoặc thân robot chỉ rộng 10 cm.
- Node tránh vật cản hãng dùng scan để chọn tiến/rẽ; không có bằng chứng hoàn tất quét phủ toàn phòng trong file `app/app/lidar_controller.py`. MAR giữ planner frontier để xác định phần chưa khám phá.

## Những gì đã sửa

1. Điểm LiDAR ở sau robot không còn bị tính thành khoảng cách âm phía trước. Điểm sau vẫn được kiểm tra khi quay và theo ngưỡng dừng quanh LiDAR.
2. Tự quét dừng khi khoảng cách LiDAR hợp lệ nhỏ hơn hoặc bằng 0,10 m ở bất kỳ hướng nào. Mã lý do: `LIDAR_OBSTACLE_WITHIN_10CM`. Đây là khoảng cách từ cảm biến, không phải từ mép thân.
3. Preset thân robot không xe kéo dùng trước/sau 0,12 m, trái/phải 0,10 m tính từ `base_footprint`: bao 24 × 20 cm, có đệm nhỏ so với thông số thân 21,2 × 17,1 cm bạn cung cấp. Vị trí tâm và đồ gắn nhô ra cần đối chiếu TF/thực tế. Không dùng preset này khi gắn lại xe kéo.
4. Nếu planner báo footprint không lọt trên map, giữ đứng yên và báo đúng nguyên nhân map; không chuyển sang chạy phản xạ. Kiểm tra footprint cả khi không còn frontier; ô vật cản nằm trong thân không được báo nhầm là đã quét xong. BLOCKED không phải quét xong và không tự báo coverage complete.
5. Log mới `AUTONOMY_STOP` ghi footprint, khoảng cách LiDAR gần nhất, khoảng hở trước và lý do planner để phân biệt lỗi scan với lỗi map.
6. Phép chiếu scan dùng kiểm chứng AMCL tuân theo TF, gồm cả trường hợp mặt phẳng LiDAR bị lật. Không tự đặt góc lắp cảm biến dựa trên hình robot.
7. Thêm `check_robot_geometry.py` chỉ đọc: cấu hình, footprint Nav2 trong YAML, polygon footprint đang phát, raw scan, TF, ô map nằm trong thân và trạng thái định vị.

Ngưỡng 10 cm của bản này áp dụng cho tự quét. Điều khiển tay và MOVE vẫn dùng gate/giới hạn đang có. Kiểm tra thân và vùng quay có thể dừng sớm hơn ngưỡng 10 cm để tránh va chạm; giảm ngưỡng cảm biến không làm thân robot nhỏ lại. Các giá trị nhỏ hơn `range_min` của driver không phải phép đo khoảng cách hợp lệ.

## Bằng chứng map đang lỗi

Snapshot BE lấy lúc 15:14 UTC chứa ô occupied cách tâm robot khoảng 5,4 cm, nằm trong thân. Thử cả footprint cũ 44 × 36 cm và footprint mới 24 × 20 cm đều bị chặn trên snapshot đó. Không thể kết luận chỉ do footprint quá lớn.

Cần phân biệt:

- `raw_scan.hits_inside_body > 0`: kiểm tra dây, đồ gắn, hướng/offset TF và phản xạ vào chính thân robot.
- Raw scan sạch nhưng `map_body.occupied_cells_inside_body > 0`: kiểm tra vệt map cũ, SLAM/odom/TF lệch hoặc pose không khớp map.
- Cả hai bằng 0 nhưng planner vẫn báo footprint: kiểm tra vùng quay + margin, biên unknown và độ thô của lưới planner. Planner dùng bao tròn quanh thân và inflation bảo thủ.

Không xóa ô đen hoặc biến unknown thành free để ép planner chạy. Chưa xác nhận tự quét thành công trên phần cứng sau bản sửa này.

## Cài bản vá

Đây là bản bổ sung trên nền ManualDrive đã cài. Installer kiểm tra hash phiên bản nguồn đã biết; gặp `CODE_BASELINE_MISMATCH` sẽ dừng trước khi ghi. Không dùng cách ép ghi đè khi có thay đổi lạ.

Trước khi cài: dừng tự chạy/đưa joystick về giữa; kết thúc và lưu job trên UI nếu đang quét. Ở terminal chạy robot, Ctrl+C và chờ tất cả dòng STOP xong.

### 1. PowerShell trên laptop

```powershell
scp 'C:\Users\LENOVO\Downloads\MAR_AutonomyGeometryFix_PI_2026_10_23.zip' pi@192.168.1.22:/home/pi/
ssh pi@192.168.1.22
```

### 2. HOST Pi

```sh
sudo docker cp /home/pi/MAR_AutonomyGeometryFix_PI_2026_10_23.zip MentorPi:/tmp/
sudo docker exec -it -u ubuntu -w /home/ubuntu/mediassist-mqtt-2026-10-08 MentorPi zsh
```

### 3. Trong container MentorPi

```sh
cd /home/ubuntu/mediassist-mqtt-2026-10-08
python3 -m zipfile -e /tmp/MAR_AutonomyGeometryFix_PI_2026_10_23.zip /tmp/MAR_AutonomyGeometryFix_2026_10_23
pi/.venv/bin/python /tmp/MAR_AutonomyGeometryFix_2026_10_23/install_pi_patch.py
pi/.venv/bin/python /tmp/MAR_AutonomyGeometryFix_2026_10_23/install_pi_patch.py --apply
pi/.venv/bin/python tools/configure_autonomy_geometry.py --trailer-removed --apply
pi/.venv/bin/python tools/check_robot_geometry.py --config-only
```

Helper chỉ chỉnh hình học tự quét, giữ chế độ mapping/navigation đang chọn, tốc độ joystick, camera, MQTT/TLS/token, map và endpoint. Không bật lại autonomy nếu trước đó đang tắt. Không thay footprint Nav2 YAML hiện có. Mỗi lần thay cấu hình có file `.bak` kèm thời gian; code có backup tại `pi/state/code-backups/`.

Để tiếp tục quét map mới, khi cấu hình đang ở mapping:

```sh
zsh tools/robot.zsh run
```

Giữ terminal mở. Trong container khác, để robot đứng yên, chưa bấm tự quét:

```sh
cd /home/ubuntu/mediassist-mqtt-2026-10-08
pi/.venv/bin/python tools/check_robot_geometry.py --seconds 6
tail -n 30 pi/state/stack/worker.log
```

Gửi output JSON này để kiểm tra TF/scan/map thực tế. Tool không phát vận tốc, không reset map, không đổi config. Một TF `map → base_footprint` tồn tại trong report không tự được coi là đã định vị đúng; report đánh dấu `quality_verified: false`.

## Footprint khác vị trí ban đầu thế nào?

- **Footprint**: hình chiếm chỗ của thân quanh tâm robot, trong hệ `base_footprint`. Dùng để tránh va chạm.
- **Pose**: tâm đó nằm ở x/y nào và quay hướng nào trong hệ `map`. Dùng để vẽ marker và đi tới endpoint.
- **Origin YAML**: vị trí gốc của raster map. Không phải vị trí hiện tại hoặc vị trí khởi động của robot.
- **Endpoint**: điểm đích; chọn endpoint không đồng nghĩa robot đang đứng ở đó.

Footprint tự quét nằm trong `mapping.autonomy.footprint`. Footprint Nav2 nằm trong `local_costmap` và `global_costmap` của YAML navigation. Hai nơi chưa tự đồng bộ. Tool kiểm tra mới hiển thị cả hai; muốn thay Nav2 phải tạo YAML riêng với công cụ có sẵn `pi/tools/configure_navigation.py nav2-params` và dùng đúng file đó khi chạy navigation.

## Luồng định vị trên map đã lưu

1. HTML tải PNG + metadata để hiển thị. Thao tác này chưa nạp map vào Pi.
2. BE kích hoạt và gán map cho robot. Pi tải bundle YAML/PGM/metadata và navigation profile qua `tools/install_be_map.py`. Pi kiểm tra map ID/hash/geometry; cài xong phải cấu hình lại localization gắn với map đó.
3. Chạy **navigation**, không chạy SLAM mapping đồng thời. `map_server` nạp map tĩnh; AMCL dùng LiDAR + odom + TF.
4. Khởi tạo một trong hai cách:
   - `seed`: đưa vị trí gần đúng `x_m, y_m, yaw_rad` của robot đang đứng trên map. Pi gửi `/initialpose` khi robot đứng yên và điểm khởi tạo thuộc vùng free.
   - `global`: Pi gọi `/reinitialize_global_localization`, rải giả thuyết khắp vùng free. Không đưa tọa độ giả `(0,0)`. Node kiểm chứng không tự phát lệnh quay để tìm vị trí.
5. AMCL ước lượng phép biến đổi `map → odom`; odometry cung cấp `odom → base_footprint`; TF cảm biến cung cấp `base_footprint → lidar_frame`.
6. Pi chỉ công nhận `LOCALIZED` khi map/hash đúng, map_server/AMCL active, scan/odom/pose còn mới, scan khớp map, covariance đủ nhỏ, TF/AMCL nhất quán và ổn định qua nhiều scan.
7. Pi gửi pose được kiểm chứng lên BE. HTML chỉ vẽ marker khi pose mới và đúng map ID/hash. Workflow MOVE dùng endpoint x/y/yaw qua Nav2; mất định vị khi đang MOVE sẽ đi theo đường dừng.

Mặc định kiểm chứng trong code hiện tại: sai số chuẩn XY <= 0,15 m; yaw <= 0,20 rad; scan match >= 0,60; known fraction >= 0,50; >= 40 tia hợp lệ; ổn định >= 3 giây và >= 8 scan khác nhau. Chế độ global còn yêu cầu >= 85% trọng số hạt gần cùng giả thuyết. Đây là tiêu chí nhận định vị, không phải cam kết độ chính xác tuyệt đối.

Code đã có `tools/configure_localization.py --mode seed` và `--mode global` để tạo file AMCL/Nav2 riêng. Cần `--nav2-params` là YAML navigation đúng với robot và `--output-nav2-params` là file mới. Chế độ seed còn cần x/y/yaw thực tế. Không cung cấp tọa độ mẫu để copy nhầm vào robot.

Sau khi navigation đã được cài và chạy, kiểm tra trong container:

```sh
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
timeout 8s ros2 topic echo /mediassist/localization/status --once --full-length
ros2 param get /local_costmap/local_costmap footprint
ros2 param get /global_costmap/global_costmap footprint
tail -n 40 pi/state/stack/localization.log
```

Trong mapping, `/mediassist/localization/status` có thể chưa tồn tại: đó là hai chế độ khác nhau. Lần đọc BE gần nhất trong phiên kiểm tra này: mapping đã `SAVED`, `currentMapId=null`, pose `fresh=false`; chưa phải robot đang được AMCL định vị trên map tĩnh.

## Những phần còn thiếu ở giao diện/BE

Hiện trang có xem map, gán map trên BE, endpoint và workflow MOVE. Chưa có nút/API chuyên biệt để gửi initial pose từ thao tác bấm/kéo trên map tới Pi, cùng ACK và trạng thái AMCL chi tiết. Chưa có luồng tự động đầy đủ “gán map trên BE → tải/nạp map trên Pi → khởi tạo localization” chỉ bằng nút HTML. Bản này không giả lập rằng các phần đó đã được triển khai.

Bước tích hợp phù hợp: thông điệp khởi tạo có robot/map ID/hash, x/y/yaw, command ID và hạn dùng; Pi chỉ nhận khi đứng yên/không có MOVE; trả ACK rồi phát trạng thái VERIFYING/LOCALIZED/FAILED lên BE. Cần triển khai cả BE/HTML/Pi trước khi dùng thay công cụ cấu hình hiện tại.

## Phòng nhiều đồ và phòng trống

AMCL so hình dạng scan với tường/vật cản trong map. Đồ đạc mới có thể được costmap coi là vật cản động trong khi định vị dựa vào phần tường/đặc trưng còn ổn định. Đổi bố trí quá nhiều cần kiểm tra lại scan match và có thể quét map mới.

Phòng trống nhưng có tường/góc bất đối xứng vẫn định vị được. Phòng rộng, đối xứng hoặc thiếu đặc trưng có nhiều vị trí cho scan giống nhau; global không đảm bảo tìm đúng khi chỉ đứng một chỗ. Khi đó cần vị trí gần đúng, điểm xuất phát/dock đã biết hoặc mốc bổ sung. Không cho robot chạy tự do chỉ vì nhận được TF.

## Kiểm thử và giới hạn

Kiểm thử offline bao gồm điểm LiDAR phía sau, ngưỡng 10 cm bốn hướng, plan BLOCKED không chạy phản xạ, map/robot quay khác hướng, occupied nằm trong thân, TF cảm biến và quy trình AMCL khởi tạo/mất định vị. Xem `verification.json` trong folder chuẩn bị trên laptop.

Installer chỉ cập nhật code đã kiểm tra hash và có rollback khi config không load được. Không thay BE đang chạy hoặc tự điều khiển robot. Cần output chẩn đoán trên Pi để chốt nguyên nhân các ô occupied trong thân.
