# Sửa MentorPi bằng VS Code — 25/10/2026

Source đã sửa trên laptop. Pi đã tắt theo xác nhận của bạn nên **chưa cài lên Pi, chưa chạy thử trên sàn**. BE đang chạy vẫn dùng image cũ cho đến khi bạn chạy bước cập nhật BE dưới đây. Không cần ZIP.

## 1. Kết nối đúng nơi

Bật robot. Trên laptop mở VS Code, nhấn `Ctrl+Shift+P` → `Remote-SSH: Connect to Host…` → nhập:

```text
ssh pi@192.168.1.28
```

Chọn **Linux**, đăng nhập bằng mật khẩu Pi. Góc dưới trái phải hiện `SSH: 192.168.1.28`.

Trong Terminal **HOST Pi**, chạy:

```sh
sudo docker start MentorPi
```

Tiếp theo `Ctrl+Shift+P` → `Dev Containers: Attach to Running Container…` → chọn `MentorPi`. Nếu không có lệnh, cài extension **Dev Containers** của Microsoft, ID `ms-vscode-remote.remote-containers`, rồi mở lại cửa sổ SSH.

Trong cửa sổ đã attach container, **File → Open Folder**:

```text
/home/ubuntu/mediassist-mqtt-2026-10-08
```

Đây mới là code robot đang sử dụng. `/home/pi` là thư mục của HOST; sửa các bản stage cũ ở đó không cập nhật runtime trong container.

## 2. Cập nhật source đã sửa, có sao lưu

Dừng terminal `robot.zsh run` bằng `Ctrl+C`, chờ toàn bộ dòng STOP. Không mở worker hay server camera thứ hai.

Trong **PowerShell trên laptop**, chạy:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File 'C:\Users\LENOVO\Downloads\MAR_Mecanum_Autonomy_2026_10_25\Update-PiRuntime.ps1' -PiIp 192.168.1.28 -TuneBareMecanum
```

Lệnh chuyển thư mục code qua SCP, kiểm tra SHA-256 của bản hiện tại, sao lưu và cài vào container. Có thể hỏi mật khẩu SSH nhiều lần. Không tự chạy robot. Nếu hiện `CODE_BASELINE_MISMATCH`, file đã khác bản đối chiếu: dừng và dùng VS Code Compare với file tương ứng trong `payload/pi/`, không ghi đè cưỡng bức.

`-TuneBareMecanum` dùng cho thử nghiệm **đã tháo xe kéo**, giữ kích thước và các xác nhận hardware sẵn có. Nó đặt tự quét tối đa **0,18 m/s theo vector XY**, **0,6 rad/s**, không đổi tốc độ joystick, IP, CA, token, map hay journal. Giới hạn phần cứng thấp hơn vẫn có hiệu lực. Bạn có thể bỏ switch này để chỉ cập nhật code.

Nếu lần cài trước đã in `PI_PATCH_APPLIED` và `WORKER_CONFIG_OK` rồi mới lỗi `No module named 'ament_index_python'`, code đã cài xong. Đó là môi trường ROS chưa được source ở bước tuning, không phải bằng chứng thiếu package camera. Không cần chép lại source. Trong terminal VS Code đang attach MentorPi, chuyển sang user ubuntu bằng `sudo -iu ubuntu`, rồi chạy riêng:

```sh
source /home/ubuntu/.zshrc
cd /home/ubuntu/mediassist-mqtt-2026-10-08
ros2 pkg prefix web_video_server
pi/.venv/bin/python tools/configure_mecanum_runtime.py --trailer-removed --apply
```

Dừng stack trước khi áp cấu hình. Chỉ chạy lại `zsh tools/robot.zsh run` sau khi thấy `MECANUM_RUNTIME_APPLIED`. Script PowerShell trong bản v2 đã nạp ROS trước bước tuning này.

Sao lưu code nằm trong `pi/state/code-backups/<timestamp>/`; file cấu hình cũ được giữ cạnh `pi/config/mqtt.worker.json` với đuôi `.bak`. Installer in chính xác đường dẫn sao lưu. Để hoàn tác code, dùng đường dẫn installer staging vừa chạy và:

```sh
pi/.venv/bin/python /tmp/mar-mecanum-THOI_GIAN/install_pi_patch.py --rollback /home/ubuntu/mediassist-mqtt-2026-10-08/pi/state/code-backups/THOI_GIAN_BACKUP
```

Thay hai đường dẫn bằng giá trị thực trong output; dừng stack trước. Hoàn tác code không hoàn tác cấu hình. Nếu cần hoàn tác tuning, khôi phục đúng file `.bak` vừa tạo trong khi stack dừng.

## 3. Các file để sửa trong Explorer của VS Code

Các đường dẫn dưới đây tính từ folder container đã mở:

| File | Vai trò / phần đã sửa |
|---|---|
| `pi/config/mqtt.worker.json` | Cấu hình đang dùng: `mapping.autonomy`, `mapping.camera`, `mapping.scan_normalization`. Không chia sẻ nguyên file vì có đường dẫn thông tin xác thực. |
| `tools/configure_mecanum_runtime.py` | Preset tuning có kiểm tra hardware và sao lưu cấu hình; nên dùng thay vì gõ nhầm JSON. |
| `pi/mediassist/navigation_setup.py` | DWB và velocity smoother lấy từ YAML hãng; bật Y riêng cho Mecanum, giới hạn gia tốc và số mẫu để tránh tăng tải quá mức. |
| `tools/mapping_nav2_parameters.py` | Tạo YAML Nav2 cho tự quét từ cấu hình hãng; không sửa trực tiếp file hãng. |
| `pi/mediassist/ros_adapter.py` | Giữ đủ `linear.x`, `linear.y`, `angular.z`; giới hạn tốc độ chéo theo tổng XY. Dùng chung cho MOVE tới endpoint. |
| `pi/mediassist/mapping.py` | Chọn một nguồn điều khiển, đưa vận tốc ba trục qua gate, giữ lease cho đi ngang, chỉ tạo thumbnail RGB khi được bật. |
| `pi/mediassist/autonomous_mapping.py` | Giảm tốc theo hành lang phía robot đang dịch chuyển; tường trước không tự triệt tiêu hướng đi ngang đang thoáng. |
| `pi/mediassist/mapping_control.py` | Làm mượt vector tự chạy; giảm giới hạn vì vật cản có hiệu lực ngay. |
| `pi/mediassist/frontier_mapping.py` | Chọn điểm tiếp cận vùng chưa biết; tìm tiếp sau vùng điểm quan sát đã loại, không kết luận hết đường quá sớm. |
| `pi/mediassist/mapping_navigator.py` | Gửi goal Nav2, xác nhận kết thúc/hủy, chờ cập nhật map và chọn điểm tiếp theo. |
| `pi/mediassist/localization_node.py` | Theo dõi AMCL, yêu cầu cập nhật khi đứng yên; sửa timeout khóa READY dù bằng chứng mới đã hội tụ. |
| `pi/mediassist/localization.py` | Kiểm tra vị trí/scan/covariance/particle; tra khoảng cách vật cản theo bảng dựng một lần để giảm CPU. |
| `tools/run_navigation_session.py` | Tạo runtime riêng cho map đã lưu, dùng cùng giới hạn vận tốc và đường Y; không ghi đè base config. |

Sửa file bằng `Ctrl+S`. Dừng rồi chạy lại stack để nạp code/cấu hình mới. Không sửa `pi/state/stack/mapping-nav2.yaml` làm cấu hình lâu dài vì file đó được tạo lại.

Nếu muốn đổi tốc độ sau này, trong terminal **container** tại folder project, khi stack đang dừng:

```sh
pi/.venv/bin/python tools/configure_mecanum_runtime.py --trailer-removed --speed 0.18 --angular 0.6
pi/.venv/bin/python tools/configure_mecanum_runtime.py --trailer-removed --speed 0.18 --angular 0.6 --apply
```

Dòng đầu chỉ xem kế hoạch. Các số này là trần, robot vẫn giảm tốc theo LiDAR và quỹ đạo Nav2. Bản này không bật lùi tự động; tránh bằng tiến, đi ngang và quay. Lùi joystick vẫn theo cấu hình và guard phía sau sẵn có.

## 4. Cập nhật HTML trên BE

Source đã được sửa ở cả BE chạy trên laptop và bản Git mirror. API và MQTT schema không đổi. Trang mới chờ xác nhận START; START thất lạc sẽ được gửi lại với cùng run ID trong khoảng chờ hữu hạn, không gia hạn một run chưa bắt đầu và không tự khởi động lại run đã kết thúc.

Trong **PowerShell laptop**, khi không có job đang chạy/cần phục hồi:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File 'C:\Users\LENOVO\Downloads\MAR_Mecanum_Autonomy_2026_10_25\Update-BeUi.ps1'
```

Script đọc số job đang giữ bằng giao dịch chỉ đọc, build/recreate riêng backend, reload nginx để nhận IP container mới. Không đổi volume, CA hoặc mạng. Sau đó mở `http://127.0.0.1:8080/mapping.html` và nhấn `Ctrl+F5`.

Files BE cần sửa trong VS Code laptop:

```text
C:\Users\LENOVO\Downloads\MAR_EMQX_Demo\BE_MAR\src\main\resources\static\mapping-autonomy.js
C:\Users\LENOVO\Downloads\MAR_EMQX_Demo\BE_MAR\src\main\resources\static\mapping.html
```

`mapping.html` hiện có bản JS nhúng giữa `BEGIN_AUTONOMY_SCRIPT` và `END_AUTONOMY_SCRIPT`; hai bản đã được cập nhật đồng nhất. Nếu tự sửa logic, phải đồng bộ cả hai.

## 5. Chạy tự quét

Trong terminal **container**. Nếu `whoami` trả `root`, chạy `su - ubuntu` trước; rồi:

```sh
cd /home/ubuntu/mediassist-mqtt-2026-10-08
zsh tools/robot.zsh run
```

Giữ terminal mở. Chờ `STACK_READY` và `BE MQTT session ready`. Trên trang HTML, tạo/theo dõi một job quét đang MAPPING, đưa cần joystick về giữa, rồi bấm **Bắt đầu tự quét**. Chạy stack tự nó chưa ra lệnh tự chạy.

Luồng: HTML → BE → MQTT START/RENEW có ACK → Pi chọn frontier → Nav2 DWB tạo XY/yaw → MAR giảm tốc/giới hạn → safety gate → chassis hãng. LiDAR/odom/TF còn mới và watchdog vẫn bắt buộc. Đây là luồng SLAM + khám phá; AMCL chỉ dùng ở chế độ map đã lưu.

Camera giữ đúng đường hãng:

```text
http://192.168.1.28:8080/stream?topic=/ascamera/camera_publisher/rgb0/image&type=mjpeg&width=640&height=480&quality=60
```

Không mở thêm `ros2 run web_video_server ...` hoặc server Python 8081. Driver ASC và MS200 vẫn là của hãng. Chỉ nhánh SLAM được chuẩn hóa 360 ô góc để tránh lỗi số tia biến đổi; không giả tăng Hz LiDAR và không sửa dữ liệu scan raw dùng tránh vật cản/AMCL. Thumbnail MQTT mặc định tắt trong preset này; nếu cần ảnh qua BE ở mạng khác, có thể bật lại `mapping.camera.mqtt_preview_enabled=true`, chấp nhận thêm tải giải mã RGB.

Trong terminal container khác, kiểm tra khi đang chạy:

```sh
cd /home/ubuntu/mediassist-mqtt-2026-10-08
tail -n 40 pi/state/stack/worker.log
tail -n 30 pi/state/stack/navigation.log
ros2 param get /controller_server FollowPath.max_vel_y
ros2 topic info /ascamera/camera_publisher/rgb0/image --verbose
ros2 topic info /scan_raw --verbose
```

Với preset, `FollowPath.max_vel_y` phải là `0.18`. RGB chỉ cần một publisher `camera_publisher`; nhánh video là native server, không còn subscriber thumbnail Python `mediassist_runtime` trong preset này. `scan_raw` phải có đúng một driver publisher. Nhiều subscriber scan phục vụ SLAM, gate, Nav2 là các chức năng khác nhau; không phải nhiều driver trùng.

## 6. Dùng map đã lưu và xác định vị trí

Kết thúc job quét/lưu đúng luồng trên UI, dừng `robot.zsh run` và chờ STOP. Tạo **phiên map tạm mới** trên UI, sau đó chạy nguyên lệnh `navigation_session.zsh --session-id ...` mà UI vừa cấp. Không dùng lại ID phiên cũ đã đóng/hết hạn. Không chạy hai stack mapping và navigation đồng thời.

Chọn **Yêu cầu tự tìm vị trí toàn cục**. AMCL so scan với map, kiểm tra covariance, độ hội tụ particle, TF và nhiều scan liên tiếp trước khi READY; tải ảnh map không tự xác định vị trí thật. Log lần trước có scan-match 99,5% nhưng code timeout vẫn ép false: phần khóa đó đã sửa, vẫn phải đủ toàn bộ điều kiện kiểm chứng.

Nếu phòng có nhiều vị trí giống nhau, chọn **vị trí thật ước lượng** và kéo hướng thật của robot trên map để seed AMCL, rồi chờ READY. Mũi tên khởi tạo không phải lệnh robot chạy tới vị trí đó. MOVE tới endpoint là một job riêng, chỉ được gửi sau khi READY trên đúng map/session; đích gồm x, y và yaw.

Kiểm tra trong terminal container khác:

```sh
tail -n 40 pi/state/stack/localization.log
ros2 lifecycle get /amcl
ros2 topic echo /mediassist/localization/status --once --full-length
```

AMCL không chạy trong mode tự quét là đúng. Khi Pi/stack đã tắt, `Node not found` cũng là kết quả dự kiến. Nếu có `MAPPING_CONTROL_EXPIRED_OR_CLOCK_SKEW`, kiểm tra/đồng bộ UTC của Windows và HOST Pi; không bỏ kiểm tra hết hạn lệnh. SSH và VS Code có thể mở đồng thời, không gây ra lỗi đó.

## Đã kiểm thử / còn phải kiểm thử

156 kiểm thử Python và 17 kiểm thử JS đã qua. Đã kiểm tra đi ngang thật qua vòng mapping → Twist → lease, giới hạn vector chéo, dừng khi mất dữ liệu, lựa chọn frontier sau điểm đã đi tới, hội tụ AMCL sau timeout, START/ACK thất lạc và không tự tiếp tục sau STOP.

Chưa đo FPS/CPU hoặc kiểm thử tránh vật cản trên robot với bản mới vì Pi đã tắt. 0,18 m/s là cấu hình để thử, không phải số đo tốc độ thực. Map vẫn giữ vật cản đen và vùng chưa biết xám; hoàn tất là hết frontier đáng kể có thể tiếp cận trong thời gian xác nhận, không phải xóa màu đen/xám. Khi chưa có đường an toàn thì robot dừng và báo nguyên nhân, không coi là đã quét xong.
