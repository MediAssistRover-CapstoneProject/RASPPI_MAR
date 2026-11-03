# Bản chuẩn bị Pi 0.14.3 và BE — 03/11/2026

Đã sửa source trong hai folder Git và chuẩn bị file để cài sau. Chưa cài bản này lên Pi, chưa restart BE, chưa chạy robot, chưa commit/push. BE đang dùng AWS MySQL; bản sửa không đổi DB, Robot ID, IP, MQTT credential, HTTPS, TLS hoặc CA.

## Luồng chạy

1. Nạp map và định vị AMCL một lần khi mở navigation. Giữ phiên này để chạy nhiều workflow/job và nhiều endpoint; không mở lại phiên chỉ vì muốn chạy tuyến khác.
2. Quyền phiên, map/hash và revision khởi tạo được gửi cùng `SERVER_HEARTBEAT` hiện có. Pi nghe MQTT và đọc cache cục bộ; không gọi HTTPS hỏi quyền phiên mỗi 2 giây khi MQTT đang cung cấp quyền mới.
3. MOVE kiểm tra danh tính lệnh và cảm biến hiện có tại Pi. Không gọi global localization hoặc phát lại initial pose trước mỗi job/endpoint. AMCL vẫn hiệu chỉnh map→odom, odometry/TF/LiDAR vẫn cập nhật vị trí khi robot chạy. Không đóng băng tọa độ ban đầu.
4. Khi hai bên hỗ trợ `move-start-commit-v1`, Pi chỉ gửi một `COMMAND_ACK` kết hợp nhận/bắt đầu MOVE. BE ghi nhận bền vững rồi trả `EVENT_ACK`; không chờ thêm một vòng `STEP_STARTED`. Kết quả cuối và chống chạy lặp vẫn giữ. Các step chưa thương lượng cơ chế này giữ giao thức tương thích cũ.
5. Pi tính/kiểm tra path, footprint và restricted zone tại chỗ. Upload path về BE chạy nền, một request đang xử lý và tối đa một bản path mới nhất đang chờ. Lỗi UI/DB hiển thị path không chặn vận tốc. Ảnh path là quan sát, không phải giấy phép chạy bánh.
6. Nav2/RPP tiếp tục bám đường và cua tiến; có thể cập nhật đường khi vật cản thay đổi. Không gửi một job motor mới cho mỗi điểm trên đường. Endpoint có yêu cầu hướng vẫn dùng cơ chế căn hướng cuối hiện có; không bỏ kiểm tra va chạm khi xoay.

Đổi map, revision khởi tạo, đóng/hết hạn phiên hoặc khởi động lại tiến trình vẫn cần kiểm tra đúng bối cảnh. Một bản READY cũ không đủ để cho chạy sau khi robot bị nhấc sang chỗ khác. Đổi mạng chỉ sửa cấu hình mạng; không tự tạo thêm BE/worker cùng Robot ID.

## Các mốc chờ

| Phần | Giá trị/hành vi |
|---|---|
| HELLO | 20 giây tối đa, nhận ACK sớm thì tiếp tục ngay |
| Heartbeat MQTT | 3 giây; không đợi nhịp này mới xử lý command |
| Mất liên lạc BE | Giữ watchdog hiện có 15 giây |
| Quyền phiên MQTT | Cache tối đa 15 giây; đọc lại cache không gia hạn |
| HTTPS quyền phiên | Dự phòng cho BE cũ/mất cập nhật MQTT; timeout request 4 giây |
| Map/lifecycle Nav2 đã xác minh | Dùng cache thành công 5 giây, giảm RPC lặp giữa các chặng |
| Path lên BE | Bất đồng bộ; không nằm trên đường cho phép motor |

ACK ứng dụng là cần thiết để biết Pi đã nhận đúng operation và tránh thực hiện hai lần sau reconnect. MQTT PUBACK chỉ xác nhận vận chuyển gói, không thay thế kết quả nhận/thực hiện step. Chờ ACK là giới hạn tối đa, không phải sleep bắt buộc. Trạng thái UI chậm không đồng nghĩa MQTT đã ngắt.

## Khoảng hở 2 cm

`pi/mediassist/footprint.py`: `NAVIGATION_PADDING_M = .02`, dùng chung cho Nav2, zone và kiểm tra endpoint. Công cụ riêng bên dưới đặt stop margin 0,02 m, near-slow margin 0,05 m tính từ thân; giữ nguyên kích thước thân và các trần vận tốc hiện có. Không áp dụng cho xe kéo.

Phanh động theo vận tốc, tuổi scan và độ trễ vẫn giữ; khi chạy nhanh phải giảm tốc trước vùng 4–5 cm. Khoảng 2 cm là tham số hình học, không phải cam kết sai số vật lý 2 cm; lưới map 2,5 cm, LiDAR và quãng phanh có sai số. Ngưỡng đo gần của LiDAR không được sửa giả thành 2 cm từ tâm cảm biến. Vùng cấm và giới hạn tốc độ vẫn có hiệu lực.

## Cài sau tại trường

Lưu/kết thúc job rồi dừng stack trước khi cài. Không khởi chạy test MOVE tự động. Các lệnh PowerShell chạy trên laptop; các lệnh `pi/.venv/...` chạy trong container MentorPi.

### 1. Đồng bộ source BE, giữ DB AWS và các secret

PowerShell trên laptop:

```powershell
Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass -Force
$project = 'C:\Users\LENOVO\Downloads\MAR_EMQX_Demo\BE_MAR'
$gitBe = 'C:\Users\LENOVO\OneDrive\Tài liệu\GitHub\BE_MAR'
$stage = Join-Path $project 'output\pi-navigation-continuity'

& "$gitBe\deploy\Apply-NavigationContinuity.ps1" -StageDirectory $stage -ProjectDirectory $project
& "$gitBe\deploy\Apply-NavigationContinuity.ps1" -StageDirectory $stage -ProjectDirectory $project -Apply
```

Script kiểm tra hash toàn bộ trước khi chép, sao lưu source cũ và không tự restart. Nếu báo `SOURCE_CHANGED`, giữ nguyên file và đối chiếu thay đổi; không xóa kiểm tra hash.

Sau khi chép thành công và stack Pi đã dừng, rebuild đúng BE hiện có, giữ project Docker và DB AWS:

```powershell
Set-Location -LiteralPath $project
docker compose -p mar-emqx-demo --env-file .env --env-file .env.aws -f compose.yaml -f deploy/compose.aws.yaml --profile production up -d --build --no-deps backend
docker logs --tail 50 mar-emqx-demo-backend-1
```

Đừng bật thêm BE khác dùng chung MQTT identity. Không chạy `down -v`, không import lại DB.

### 2. Chép các file Pi, không chép test hoặc secret

PowerShell trên laptop, thay IP bằng IP SSH lúc ở trường:

```powershell
$piIp = '10.3.93.158'
ssh "pi@$piIp" 'mkdir -p /home/pi/mar-navigation-continuity'
scp -r "$stage\files" "$stage\manifest.json" "$stage\install_navigation_continuity.py" "pi@${piIp}:/home/pi/mar-navigation-continuity/"
ssh "pi@$piIp" 'sudo docker cp /home/pi/mar-navigation-continuity MentorPi:/tmp/'
ssh -t "pi@$piIp" 'sudo docker exec -it -u ubuntu -e HOME=/home/ubuntu -w /home/ubuntu/mediassist-mqtt-2026-10-08 MentorPi zsh'
```

Nếu đã mở đúng container bằng VS Code, bỏ qua bước SSH/Docker exec, mở Terminal tại folder `/home/ubuntu/mediassist-mqtt-2026-10-08` rồi chạy:

```sh
cd /home/ubuntu/mediassist-mqtt-2026-10-08
pi/.venv/bin/python /tmp/mar-navigation-continuity/install_navigation_continuity.py
pi/.venv/bin/python /tmp/mar-navigation-continuity/install_navigation_continuity.py --apply
pi/.venv/bin/python tools/configure_motion_clearance.py --trailer-removed --stop 0.02 --slow 0.05
pi/.venv/bin/python tools/configure_motion_clearance.py --trailer-removed --stop 0.02 --slow 0.05 --apply
pi/.venv/bin/python -c 'import sys; sys.path.insert(0,"pi"); from mediassist.step_contract import VERSION; print(VERSION)'
```

Lệnh cuối cần in `0.14.3`. Dùng đúng `pi/.venv/bin/python`, không dùng Python hệ thống thiếu dependency. Installer chặn khi stack còn chạy và giữ bản sao tại `pi/state/patch-backups/navigation-continuity-*`. Cấu hình khoảng hở được sao lưu riêng bởi `write_configuration`. Nếu một lệnh báo lỗi thì dừng tại đó, không chạy tiếp cấu hình/run.

### 3. Chạy lại

Quét map: `zsh tools/robot.zsh run`. Dẫn đường trên map đã lưu: mở/chọn một phiên navigation trên HTML và dùng đúng lệnh `zsh tools/navigation_session.zsh --session-id ...` mà trang trả về; không chạy song song với `robot.zsh run`. Định vị một lần, giữ phiên để chạy các workflow/tuyến tiếp theo. Khi đổi mạng, dùng các helper cập nhật IP/HTTPS hiện có trước khi chạy; bản sửa này không đổi địa chỉ mạng.

## Kiểm tra đã thực hiện

Kiểm tra tập trung Pi: mailbox không tự gia hạn, gói trễ/khác map/khác revision, thu hồi phiên, path upload không chặn event loop, giữ độ cong khi phanh, 2 cm không đổi tốc độ/TLS; kiểm tra hồi quy ACK, HELLO, path và AMCL không tái khởi tạo. BE: 8 unit test về HELLO/authorization/path; UI: 3 test pose realtime; schema heartbeat mới và cũ đều hợp lệ. Chưa đo độ trễ hoặc xác nhận độ mượt trên robot với bản 0.14.3. Khi thử thực tế cần xem mốc nhận command, committed ACK, Nav2 nhận goal và vận tốc xuất ra; không thể hứa độ trễ bằng 0 hoặc không bao giờ dừng khi mất dữ liệu thật.
