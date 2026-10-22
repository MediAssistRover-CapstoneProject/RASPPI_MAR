# MS200: số tia thay đổi làm SLAM ngừng thêm scan

Driver đang dùng vẫn là `oradar_lidar/oradar_scan` của hãng. Sáu mẫu đo thật
ngày 22-10-2026 có 445/446/447/449 tia, góc bắt đầu và bước góc cũng thay đổi.
SLAM Toolbox đăng ký một cấu trúc laser rồi từ chối scan không khớp số tia.
Topic map phát 2 Hz có thể chỉ phát lại grid cũ. Đây là lỗi đầu vào SLAM,
không phải bằng chứng MQTT/HTML bị đứt hoặc gamepad làm ngừng quét.

## Luồng dữ liệu

```text
MS200 + driver hãng -> /scan_raw -> safety, rear guard, worker health (giữ nguyên)
                           |
                           +-> scan_normalizer -> /mediassist/scan_slam -> SLAM
```

Bật `mapping.scan_normalization={"enabled":true,"bins":360}` trong cấu hình
worker. Session manager mở một child `mediassist.scan_normalizer` trước SLAM,
theo dõi và đóng child khi stack dừng; reset graph giữ normalizer chạy.
Không mở thêm driver LiDAR, không sửa thiết bị/baudrate/USB, không bật localization.

Normalizing dùng góc thật `angle_min+i*angle_increment`, gộp về 360 bin 1 độ.
Nếu nhiều tia rơi cùng bin, giữ vật cản gần nhất. Hướng không đo được là NaN,
không tạo vùng trống giả. Tia +inf đo thật được giữ +inf; dữ liệu ngoài giới hạn,
NaN, -inf hoặc 0 dưới range_min không biến thành khoảng cách xa giả.
Header/frame/timestamp, range limits và scan_time giữ nguyên. Scan cũ/sai thứ tự
không được làm mới timestamp. `time_increment=0` vì bước gộp không chứng minh
được thời điểm đều cho từng tia đầu ra; đây không phải deskew chuyển động.

Độ phân giải góc 1 độ thô hơn một chút so với MS200 ~0,8 độ ở các mẫu đã gửi.
Độ phân giải grid map 2,5 cm/ô giữ nguyên. Không khẳng định bản vá sửa được mọi
vấn đề chất lượng map: vẫn cần odometry/TF đúng và hạn chế trượt bánh/quay nhanh.

## Kiểm tra sau khi cài

Kết thúc job cũ rồi dừng stack trước khi vá. Sau cài, chạy stack và tạo phiên
quét mới để SLAM đăng ký laser theo lưới cố định ngay từ đầu.

```sh
zsh tools/robot.zsh run
```

Terminal container khác:

```sh
tail -n 20 pi/state/stack/scan-normalizer.log
ros2 param get /slam_toolbox scan_topic
timeout 6s ros2 topic hz /mediassist/scan_slam --window 20
```

Mong đợi `SCAN_NORMALIZED raw=... fixed=360`, scan_topic là
`/mediassist/scan_slam`, nhịp gần nhịp scan gốc (~10 Hz), không phải map 2 Hz.
Xem log chỉ của engine mới để tránh nhầm warning cũ:

```sh
awk '/SLAM_GENERATION/{s=""} {s=s $0 ORS} END {printf "%s",s}' pi/state/stack/slam-engine.log
```

Không còn cảnh báo `LaserRangeScan contains ... expected ...` trong generation
mới. Cho robot đi chậm trên sàn tới vùng chưa quét để kiểm tra nội dung map và
pose thay đổi. Chỉ đếm nhịp `/map` không đủ chứng minh bản đồ mới.

## Upload map và IP BE

Log thực tế: Pi vẫn dùng `https://192.168.1.16`, laptop BE là `192.168.1.18`,
gây `No route to host` khi tự lấy device token. MQTT broker cloud là kết nối
khác, nên MQTT vẫn READY trong lúc upload thất bại.

Sau khi server certificate đã có SAN mới, copy public CA vào container tại
`/tmp/mar-be-ca.pem`, rồi dừng stack và chạy:

```sh
pi/.venv/bin/python tools/configure_be_origin.py --origin https://192.168.1.18 --ca-file /tmp/mar-be-ca.pem --apply
```

Công cụ xác minh HTTPS /actuator/health và lấy thử device token bằng secret
đang có; không in token/secret. Chỉ khi cả hai thành công mới lưu URL upload/
TTS và public CA vào cấu hình, có backup. Không đổi MQTT broker CA/credentials,
không đổi gamepad/hardware profile/motion hay storage; không xoay device secret.
Token thử không ghi ra file; cơ chế refresh hiện có dùng URL mới ở lần SAVE sau.

Nếu lệnh không có `--apply`: chỉ kiểm tra mạng và device auth, không ghi cấu hình.
Nếu fail 401/403: kiểm tra device credentials. Nếu chứng chỉ không khớp IP/CA
hoặc timeout/no route: sửa HTTPS/mạng; không tắt xác minh TLS.

Map đã SAVE trước lúc upload lỗi vẫn có thể còn ở `artifact_dir` trong
`mapping.storage_dir/session.json`; không xóa thư mục hoặc journal. Job không
được tự coi là hoàn thành upload/COMMIT chỉ vì file local tồn tại.

## Kiểm thử

Offline kiểm tra đúng 6 mẫu geometry đã gửi, tính số tia theo Karto, giữ góc vật
cản, giữ vật gần khi gộp, không làm giả vùng trống, không sửa raw scan, quản lý
child và chỉ đổi đầu vào SLAM khi bật rõ. Chưa chạy bản vá trên robot thật.

Nguồn: [SLAM Toolbox LaserRangeFinder::Validate](https://github.com/SteveMacenski/slam_toolbox/blob/humble/lib/karto_sdk/src/Karto.cpp#L193).
