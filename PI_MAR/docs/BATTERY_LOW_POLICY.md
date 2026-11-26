# Pin yếu: BE điều phối, Pi chặn điều khiển tay

BE dùng telemetry pin thật trong heartbeat để cảnh báo ở mức 10% và điều phối
hủy job, phát lời nhắc, rồi chạy MOVE đến endpoint CHARGER ở mức 5%. Pi dùng
`CANCEL_STEP`, `INTERACT/SPEAK` và `MOVE` hiện có; không tự chọn endpoint, tự tạo
job hoặc chạy đường về khi mất kết nối BE.

## Chặn joystick cục bộ

`ManualDrive` đọc trực tiếp `BatteryEstimator`, kể cả khi mất MQTT. Khi hardware
profile có `battery.enabled=true` và mẫu mới có `estimate_status=ESTIMATED`,
joystick dừng ở `percent <= manual_critical_stop_percent` (mặc định 5). Tick
điều khiển tay phát vận tốc 0, thu hồi lease và bỏ trạng thái armed của joystick.
Ngưỡng này áp dụng cho đường `ManualDrive` của Mecanum/auto và navigation manual.
Đường mapping cũ dùng bàn phím `control_source=local` hoặc joystick layout
`legacy` chưa đi qua gate pin này. Những đường đó vẫn dùng cơ chế abort/dừng
mapping hiện có của BE; không coi bản vá này là bảo vệ mọi nguồn điều khiển tay.

Điều khiển tay chỉ được phép trở lại khi có mẫu ESTIMATED mới với
`percent >= manual_rearm_percent` (mặc định 8), sau đó người điều khiển phải đưa
cần về trung tính theo gate sẵn có. Mẫu stale/invalid/warming-up/unconfigured
không kích hoạt hoặc gỡ latch pin yếu. Latch không tự hết hạn và không bị xóa
bởi việc nhả cần hay gọi reset của bộ ramp.

Hai tùy chọn đặt trong phần `battery` của hardware profile:

```json
"manual_critical_stop_percent": 5,
"manual_rearm_percent": 8
```

Ngưỡng dừng nhận số hữu hạn 0–20; ngưỡng mở lại nhận số hữu hạn 1–100 và phải
lớn hơn ngưỡng dừng. Cấu hình không bật estimator giữ hành vi cũ. Đây là gate
điều khiển tay; không từ chối MOVE của BE. Sau khi xác nhận local input đã nhả
quyền điều khiển, một cần đang giữ ở mức pin nguy cấp không chặn job về sạc.

## Độ mới và hiệu chuẩn

Thời gian mẫu lấy từ lúc Pi nhận số đo RRC, không được gia hạn bởi heartbeat.
`stale_after_sec` nhận 5–60 giây; mặc định 60. Bộ lọc mặc định median 5 mẫu và
EMA 15 giây, cần ít nhất 3 mẫu hợp lệ. Ngoài dải điện áp, thiếu mẫu hoặc stale
cho `percent=null`. Không thay bằng phần trăm giả để tiếp tục điều phối.

`curve_calibrated=false` vẫn có trạng thái ESTIMATED: đây là ước tính từ đường
cong điện áp, chưa phải phép đo dung lượng. `voltage_gain`, `voltage_offset_v`,
số cell và `cell_voltage_curve` phải phù hợp cụm pin thực; chỉ đặt
`curve_calibrated=true` sau khi đã đo và hiệu chỉnh đường cong của chính cụm pin.

`INTERACT/SPEAK` dùng `speech.default_volume` sẵn có, mặc định 100; BE có thể
đặt `data.volume` cho lời báo pin. Nếu không có `data.volume`, bước pin yếu dùng
cùng mặc định với các lời nói khác.
