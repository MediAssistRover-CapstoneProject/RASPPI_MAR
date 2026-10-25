# Tự quét, footprint, tốc độ và định vị

Trong VS Code Attach container MentorPi, mở `/home/ubuntu/mediassist-mqtt-2026-10-08`.

Chỉ sửa cấu hình khi đã Ctrl+C `robot.zsh run`, chờ STOP hoàn tất. Lệnh đặt trần tự quét Nav2:

```sh
cd /home/ubuntu/mediassist-mqtt-2026-10-08
pi/.venv/bin/python tools/configure_autonomy_speed.py --trailer-removed --mode frontier --speed 1.00 --angular 0.80 --apply
pi/.venv/bin/python tools/check_mapping_readiness.py
zsh tools/robot.zsh run
```

Chờ STACK_READY và BE MQTT session ready, tạo job mới trên mapping.html, rồi bấm tự quét. Script cấu hình không gửi vận tốc. Trần 1 m/s áp dụng cho tổng vector x/y; Nav2, góc cua, khoảng hở và tuổi scan tiếp tục hạ vận tốc thực. Từ trần trên .5 m/s, vùng giảm tốc ít nhất 1.9*speed mét. Giới hạn phanh theo scan là mô hình .6 m/s², thời gian phản ứng gồm tuổi scan + .35 giây, chưa phải đo quãng đường phanh cơ khí trên sàn thực. Tay cầm .25 m/s; endpoint trên map đã lưu giữ .25 m/s/.6 rad/s.

Muốn thử chạy phản xạ theo LiDAR tương tự chức năng né vật cản của app hãng, vẫn dựng SLAM và vẫn lưu qua cùng job:

```sh
pi/.venv/bin/python tools/configure_autonomy_speed.py --trailer-removed --mode reactive --speed 0.20 --angular 0.80 --apply
zsh tools/robot.zsh run
```

Chế độ phản xạ cho phép cùng trần 1.00 m/s/.80 rad/s, còn ví dụ 0.20 m/s dùng để thử ban đầu. Chọn hướng từ vùng LiDAR trống; khi trước sát 5 cm tính từ thân, dừng tiến rồi quay nếu đủ bao quay, hoặc đi ngang phía trống. Nếu mọi hướng chật, giữ zero và chờ scan mới, không tự nối lại một run đã bị hủy. Khoảng hở 5 cm không tính từ tâm LiDAR. Coverage tracker vẫn quan sát map và chỉ báo hoàn tất khi đủ bằng chứng; chạy phản xạ không đảm bảo khám phá mọi ngóc ngách. Có thể dừng theo nút Dừng, thời hạn hoặc stop-on-return. `--mode frontier` dùng Nav2 chọn đích khám phá có kế hoạch. Không bật đồng thời node app/lidar_controller của hãng: node đó phát vận tốc riêng. Runtime này giữ một nguồn vận tốc tới gate/watchdog.

Kiểm tra lỗi footprint: helper in file diagnostic gần nhất, vị trí SLAM lúc lỗi, kết quả ô occupied/unknown/boundary chạm thân. `ROBOT_FOOTPRINT_NOT_CLEAR_IN_MAP` nghĩa là hình bao robot hiện chạm ô cản hoặc vùng chưa biết. Không xóa ô cản hoặc thu nhỏ footprint để ép chạy. Diagnostic gần nhất có ô cản quanh x=0.093 m,y=-0.092 m theo thân; trong khi ray LiDAR mới có lúc xa hơn: cần kiểm tra môi trường thực, vật gắn trên robot và map mới sau khi scan/TF ổn định. Diagnostic cũ không đại diện cho sensor hiện tại.

LiDAR đang là MS200, SDK `SetRotationSpeed` kẹp 5..15 Hz; vendor launch đã đặt motor_speed=15. Không thể cam kết 20 Hz bằng sửa số launch. Giữ một driver; không phát lại scan cũ để giả tăng Hz. Khi robot chạy, đo:

```sh
timeout 10s ros2 topic hz /scan_raw --window 30
tail -n 35 pi/state/stack/scan-normalizer.log
tail -n 35 pi/state/stack/slam-engine.log
tail -n 35 pi/state/stack/worker.log
```

Bản này giữ tuổi scan <=.4 s, bù bằng giới hạn vận tốc theo tuổi scan. Không thay header của scan. SLAM có TF buffer 30 s, transform_timeout .5 s và scan queue 10 để đợi transform; nếu driver hoặc TF không cấp dữ liệu hợp lệ thì vẫn dừng, không coi tăng queue là bằng chứng đã sửa sensor.

Vị trí trên map: khi quét mới, SLAM dựng `map -> odom -> base_footprint`, tọa độ là của phiên quét. Khi dùng map đã lưu, dừng run quét, tải map/endpoint trên web, mở phiên navigation mới và chạy đúng lệnh navigation_session.zsh mà UI hiện. Bấm vị trí và kéo chọn hướng thực của robot rồi Gửi vị trí khởi tạo cho AMCL, hoặc dùng định vị toàn cục. Chỉ chạy MOVE khi phiên READY và pose còn mới. Đặt vị trí khởi tạo là ước lượng cho AMCL, không phải lệnh robot đi tới đó. Phiên navigation cũ d4f7087a… đã đóng và hết hạn; không tái dùng ID đó.

Endpoint cần cùng map ID/hash/frame/session. Endpoint lưu x/y/yaw, BE tạo workflow MOVE và Pi gọi NavigateToPose; không cần tọa độ z để quay camera theo hướng đích trong mặt phẳng. Các bài test navigation_session, mapping_navigator và localization kiểm tra các ràng buộc này; cần thử MOVE vật lý sau khi có READY mới.

Nguồn sửa tốc độ: tools/configure_autonomy_speed.py, pi/mediassist/configuration.py, autonomous_mapping.py, navigation_setup.py, mapping.py. SLAM: pi/mediassist/slam_session.py. Cấu hình chính: pi/config/mqtt.worker.json; hardware_profile trỏ đến bản site.motion hiện hành. Không sửa token/secret, CA, nhật ký hoặc map bằng tay.
