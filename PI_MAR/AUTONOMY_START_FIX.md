# Sửa tự quét bị chặn ở điểm bắt đầu — 23/10/2026

## Đã xác định trên Pi

- Bản GeometryFix trước đã cài, footprint trước/sau 0,12 m; trái/phải 0,10 m; tự quét bật và LiDAR stop 0,10 m.
- Lần đầu raw scan gần nhất 0,293 m, khoảng hở trước 0,089 m; planner chặn vị trí bắt đầu.
- Sau khi bạn di chuyển robot ra chỗ thoáng, raw scan gần nhất 0,422 m, khoảng hở trước 0,375 m, vùng quay thông, không có ô occupied trong thân. Planner cũ vẫn báo `ROBOT_FOOTPRINT_NOT_CLEAR_IN_MAP`.
- Replay chính map này: planner cũ chỉ nhận vùng free liên thông khoảng 1,44 m²; planner mới giữ lưới gốc 2,5 cm, nhận khoảng 2,61 m² và tìm được đường tới frontier.

## Nguyên nhân và sửa đổi

Planner cũ gộp map 2,5 cm thành ô tối thiểu 10 cm, sau đó nở vùng cấm bằng khoảng cách kiểu ô vuông. Chỉ một ô unknown/occupied cũng làm cả ô lớn bị chặn; vùng cấm được làm tròn thêm nên dễ loại nhầm điểm đứng ở map nhỏ.

Bản này:

1. Giữ độ phân giải map gốc khi mỗi chiều <= 300 ô; map lớn vẫn được giới hạn kích thước tính toán.
2. Tính khoảng cách Euclidean chính xác giữa tâm ô bằng phép biến đổi khoảng cách hai chiều O(width × height), không thêm dependency. [Nav2 cũng dùng khoảng cách Euclidean trong inflation](https://github.com/ros-navigation/navigation2/blob/humble/nav2_costmap_2d/plugins/inflation_layer.cpp).
3. Vật cản occupied giữ bao tròn quanh footprint + margin 10 cm. Unknown và biên map phải nằm ngoài toàn bộ bao thân/quay, nhưng không cộng thêm khoảng đệm 10 cm của vật cản. Cách tách này tránh coi phần chưa quét là một vật cản đã đo được. Thêm đệm theo đường chéo ô để kiểm tra cả diện tích ô, không chỉ tâm. Thân không được đi vào unknown.
4. Tìm điểm quan sát frontier với giới hạn bước tương ứng khoảng cách Euclidean; không dùng số bước Manhattan quá ngắn cho hướng chéo.
5. Chỉnh ngưỡng đến đích quan sát theo kích thước ô, tránh đứng yên khi còn cách đích nhỏ hơn 15 cm nhưng thực tế chưa tới.
6. Điểm nhìn trước phải có đường thẳng qua các ô an toàn, tránh cắt góc xuyên vùng cấm.

Không đổi vận tốc, camera, gamepad, MQTT/HTTPS, map, endpoint, cấu hình hoặc dữ liệu job. Không xóa ô vật cản để ép đường đi. Không tự phát lệnh chuyển động.

## Kiểm chứng

- 127 kiểm thử offline: 126 đạt, 1 bỏ qua, gồm map hẹp đủ chỗ, vật cản thật/unknown vẫn chặn thân hoặc đường đi, khoảng cách đối chiếu brute force, đường nhìn không cắt góc, map xoay, coverage và dừng lệnh.
- Replay tìm được `FRONTIERS_PRESENT` trên cả snapshot sau di chuyển và snapshot map mới sau restart. Phiên đầu đo trên Pi khoảng 94–101 ms/lần; số đo bản cuối được ghi thêm vào `verification.json` trên laptop.
- Snapshot map, SSH helper và thông tin đăng nhập không được đóng gói vào ZIP.

## Cài thủ công nếu cần

Đây là bản bổ sung trên GeometryFix đã cài. Nếu agent đã báo cài trực tiếp thành công trên robot này thì không cần cài lại.

Đưa joystick về giữa, kết thúc/lưu phiên quét trên UI; trong terminal robot Ctrl+C và chờ STOP hoàn tất.

PowerShell:

```powershell
scp 'C:\Users\LENOVO\Downloads\MAR_AutonomyStartFix_PI_2026_10_23.zip' pi@192.168.1.22:/home/pi/
ssh pi@192.168.1.22
```

HOST Pi:

```sh
sudo docker cp /home/pi/MAR_AutonomyStartFix_PI_2026_10_23.zip MentorPi:/tmp/
sudo docker exec -it -u ubuntu -w /home/ubuntu/mediassist-mqtt-2026-10-08 MentorPi zsh
```

Trong container:

```sh
cd /home/ubuntu/mediassist-mqtt-2026-10-08
python3 -m zipfile -e /tmp/MAR_AutonomyStartFix_PI_2026_10_23.zip /tmp/MAR_AutonomyStartFix_2026_10_23
pi/.venv/bin/python /tmp/MAR_AutonomyStartFix_2026_10_23/install_pi_patch.py
pi/.venv/bin/python /tmp/MAR_AutonomyStartFix_2026_10_23/install_pi_patch.py --apply
zsh tools/robot.zsh run
```

Installer kiểm tra hash trước khi sửa, sao lưu code vào `pi/state/code-backups/` và hoàn nguyên nếu config không load được. `CODE_BASELINE_MISMATCH` nghĩa là có code khác phiên bản đã biết; không ép ghi đè.

## Kiểm tra sau khi khởi động

Trong container khác, để robot đứng yên:

```sh
cd /home/ubuntu/mediassist-mqtt-2026-10-08
pi/.venv/bin/python tools/check_robot_geometry.py --seconds 6
tail -n 30 pi/state/stack/worker.log
```

Report có `planner_cell_m`, `clearance_radius_m`, `body_radius_m`, `seed_obstacle_clearance_m`, `seed_unknown_clearance_m`. Snapshot sau di chuyển: ô 0,025 m, bán kính thân + buffer 0,256 m, khoảng hở tới vật cản 0,283 m. Snapshot sau restart: khoảng hở vật cản 0,340 m, unknown 0,226 m; bán kính bao thân 0,156 m, nên đủ chỗ. Hai khoảng hở cần so với hai bán kính tương ứng.

Tạo phiên quét mới trên UI, chờ map/pose mới rồi bấm tự quét. Phiên tự chạy đã STOPPED không tự chạy lại chỉ nhờ RENEW; cần một lần START mới. Gần vật cản, mất dữ liệu hoặc không còn đường đủ chỗ vẫn có thể dừng. Cần kiểm tra chạy thực tế riêng, không suy ra đã đi được chỉ từ replay planner.
