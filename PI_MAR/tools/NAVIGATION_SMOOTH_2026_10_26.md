# Bản sửa dẫn đường trên map đã lưu – 26/10/2026

Trạng thái: đã kiểm thử ngoại tuyến; chưa cài lên Pi và chưa chạy thử trên sàn. Pi đang dừng vì pin yếu. Không dùng kết quả kiểm thử phần mềm làm xác nhận chạy được giữa bệnh nhân.

## Nguyên nhân đã đọc được

- Job `9ca19273-4166-4795-85a2-55b40d6611f8` đã FAILED/NAV_ABORTED, Pi xác nhận dừng. Lần đọc DB cuối không có job hoạt động/khôi phục cần hủy. Sau đó Docker BE tắt nên chưa thể đọc lại trạng thái mới.
- Log `pi/state/stack/slam.log` ghi Navfn không tìm được đường đến `(0.14, -0.35)` ở lượt lập lại đường. Chưa có bằng chứng endpoint đó đủ khoảng hở cho cả thân robot.
- Workflow MOVE trước giới hạn 0.05 m/s. Điều này khác giới hạn quét map và tốc độ tối đa của tay cầm.
- Phiên `b7c4f09d-e01b-43a9-b1b1-0ca771535dbd` đóng lúc 01:15:14 ngày 26/10 (+7) sau khi terminal kết thúc; hạn khi đó là 03:06:00. Phiên không hết hạn tại thời điểm MOVE thất bại. Vòng đời phiên gắn với một lần chạy Pi để tránh nhận lệnh của tiến trình cũ.

## Thay đổi Pi

- `pi/mediassist/navigation_setup.py`: Navfn A* trên costmap, không đi vào vùng chưa biết. Chi phí tính cả khoảng cách và khoảng hở; không cam kết đường ngắn nhất hình học trong môi trường động. Giữ polygon thân robot và DWB kiểm tra va chạm. Gradient inflation 0.45 m, hệ số 3 cho map đã lưu, vẫn cho đi qua vùng có chi phí nếu thân robot đủ hở. Đây không phải khoảng cách dừng cứng 0.45 m.
- Đồng bộ phạm vi xóa/ghi nhận LiDAR để người hoặc vật đã đi khỏi không dễ để lại vật cản ảo. Giữ marking/clearing và static map. Không phát voxel map phục vụ debug.
- `pi/config/navigate_no_recovery.xml`: lập lại đường 1 Hz; thử lại planner tối đa 2 lần, có chờ 0.5 giây và yêu cầu costmap dựng lại; controller tối đa 1 lần. Không thêm quay/lùi mù. Vật cản thật hoặc cửa quá hẹp vẫn có thể khiến MOVE thất bại.
- `pi/mediassist/ros_adapter.py`: gửi trần tốc độ của từng MOVE vào DWB qua `/mediassist/navigation_speed_limit`, tiếp tục giới hạn độ lớn vector XY ở đầu ra. Mất pose mới: phát zero và thu hồi lease ngay, giữ goal tối đa 2 giây để định vị phục hồi; quá hạn thì kết thúc lỗi. Mất lệnh Nav2: zero ngay, chờ có giới hạn cho lập lại đường. Đổi map/hash/phiên/revision/profile vẫn chặn ngay.
- `pi/mediassist/localization.py`: lần khởi tạo và mất định vị nghiêm trọng vẫn cần kiểm chứng đầy đủ. Sau một khoảng dữ liệu cũ ngắn, có thể phục hồi sau ít nhất 0.5 giây và 2 scan riêng biệt đạt đủ tiêu chí. Không cho robot chạy bằng pose cũ.
- Giữ smoothing 20 Hz, gia tốc XY 0.4 m/s², giảm tốc 0.6 m/s²; DWB Mecanum có đi ngang. Giới hạn navigation hiện tại 0.25 m/s, 0.6 rad/s; quét map/tay cầm/camera không đổi.

## Thay đổi BE/UI

Chỉ đổi tài nguyên giao diện và workflow mẫu, không thay API/DB/MQTT schema. Bấm “Chuẩn bị workflow MOVE” sau khi cập nhật để tạo hoặc chọn đúng hợp đồng mới 0.15 m/s. Job cũ và workflow đã publish không bị sửa ngược.

Mũi tên xanh chỉ dùng pose mới đúng map/hash/phiên. Khi định vị tạm mất, giữ dấu cuối màu xám tối đa 10 phút và hiện tuổi dữ liệu; dấu này không mở khóa MOVE. Xóa dấu khi đổi map/robot/phiên/revision hoặc đóng phiên. Không tự gọi global localization mỗi 5–10 phút.

## Chạy thử sau khi sạc

1. Tháo xe kéo như cấu hình đã xác nhận. Nếu gắn xe kéo có khớp, cần cấu hình lại hình học và kiểm thử riêng; polygon thân robot hiện không bao quát xe kéo.
2. Cài bản vá khi stack và worker đã dừng, cập nhật BE/UI rồi mở `http://127.0.0.1:8080/mapping.html`. Ctrl+F5.
3. Tải map ACTIVE đúng nơi đang test, chọn endpoint trên vùng trắng có đủ hở cả thân robot và góc quay; không đặt điểm sát tường/cánh cửa.
4. Mở phiên map tạm, chạy đúng lệnh `zsh tools/navigation_session.zsh --session-id ...` do UI trả về. Không mở đồng thời `robot.zsh run` khác.
5. Dùng khởi tạo gần đúng trên map hoặc yêu cầu tìm vị trí toàn cục một lần; đợi READY/LOCALIZED. Đối chiếu hướng và vị trí robot thật với map, nhất là phòng đối xứng. Đặt vị trí khởi tạo không phải lệnh di chuyển robot đến đó.
6. Chuẩn bị workflow mẫu mới, chọn endpoint, MOVE ở chỗ trống có người giám sát. Thử thẳng, góc cua, đi ngang, đứng chặn lối rồi rời đi, cửa hẹp, hủy job và mất định vị. Xác nhận zero/dừng trước khi thử lượt tiếp.
7. Xem `pi/state/stack/worker.log`, `localization.log`, `slam.log`, `gate.log`. Main Nav2 trong saved-map session đang ghi vào `slam.log`. Không khởi động thêm node chỉ để xem log.

## Luồng ứng dụng bệnh viện

Map theo từng tầng/khu → endpoint có room/zone và hướng đến → BE phân công đúng robot/phiên → step MOVE → Pi kiểm tra map/profile và định vị → A* lập đường → DWB né vật cản và velocity smoother → một cổng vận tốc/watchdog → kiểm tra lại vị trí/hướng đích → xác nhận hoàn tất step về BE. Workflow giao vật phẩm ghép thêm thao tác khóa/mở servo và xác nhận giao nhận của dự án; không mở khoang chỉ vì lệnh MOVE đã được tiếp nhận.

Trước khi dùng giữa người thật cần bản đồ đúng tầng, endpoint tiếp cận được, vùng cấm và tốc độ theo khu, khoảng hở thực đo, nút dừng người vận hành, quy trình đi qua cửa/đám đông và kiểm thử tải hàng/xe kéo. Bản vá này chưa thêm bản đồ nhiều tầng, điều khiển thang máy/cửa tự động, nhận diện người hoặc bảo đảm vị trí trong phòng đối xứng. LiDAR 2D cũng không thấy mọi vật ngoài mặt phẳng quét.

Định vị chạy liên tục bằng LiDAR + odometry + IMU hiện có; tọa độ cũ 5–10 phút chỉ làm gợi ý hiển thị, không thay pose sống. Odometry hãng hiện chưa được xác nhận có phản hồi encoder thực; cần đánh giá độ trôi khi test dài. Có thể bổ sung mốc QR/AprilTag hoặc LiDAR odometry đã kiểm chứng nếu phòng đối xứng gây nhầm.

## Các file để tự chỉnh trong VS Code

Remote SSH vào Pi → Attach to Running Container MentorPi → Open Folder `/home/ubuntu/mediassist-mqtt-2026-10-08`.

- Tốc độ workflow: BE `src/main/resources/static/navigation-move-workflow.json`, `graph.steps[0].paramsJson.max_speed_mps`; sau sửa phải chuẩn bị phiên bản workflow mới.
- Trần Nav2 của map đã lưu: Pi `tools/run_navigation_session.py`; tham số planner/controller/smoother: `pi/mediassist/navigation_setup.py`. Giữ giới hạn phần cứng và test khoảng dừng trước khi nâng.
- Định vị: `pi/mediassist/localization.py`, `localization_node.py`. Không đổi TTL thành tuổi pose được phép chạy.
- Camera vẫn dùng web_video_server của hãng, duy nhất luồng xem trực tiếp cổng 8080; không có server video mới trong bản sửa này.


## 2026-11-02: MOVE held after a turn

Observed job 325b54f6-80d4-4ad1-8a60-5e491c8b88e2 failed LOCALIZATION_LOST. Pi held wheels at 06:15:51 UTC when AMCL XY standard deviation briefly exceeded 0.15 m (about 0.155 m), then cancelled Nav2 after the existing 5 s localization hold. MQTT remained connected. This is localization readiness loss, not proof of a network outage. Stationary AMCL correction age crossed 0.8 s during cold re-verification and repeatedly reset stability.

QualityGate now preserves an already established hypothesis for bounded re-verification when covariance is at most 1.25 times the configured entry thresholds and live scan agreement is adequate. It still returns ready=false while uncertain: no uncertain pose permits motion. Recovery requires fresh healthy evidence over two distinct scans and at least 0.5 s, within the existing 10 s recovery horizon. Larger uncertainty, identity/TF faults, body conflicts, jumps or an expired hypothesis retain full initialization requirements. Sensor freshness checks and MOVE hold timeout are unchanged. This does not freeze robot coordinates or disable AMCL updates.

Validated with 23 targeted offline localization tests, including small turn-uncertainty recovery and large/expired uncertainty rejection. No physical MOVE was dispatched by this verification. The running Pi navigation session was kept at the operator's request; install localization.py only after the stack is stopped, back up and verify the existing source hash, then reopen navigation and verify the starting pose.


## 2026-11-02: installed forward tracking and endpoint heading

Installed on Pi 10.3.75.134 after confirming no robot supervisor, worker or navigation-session process was running. Backups: pi/state/patch-backups/move-localization-20261102T063437. Updated localization.py, navigation_setup.py, ros_adapter.py and tools/run_navigation_session.py. Runtime IP/TLS/CA/device credentials, journals, map assets, commissioned geometry and mapping/manual speed limits were preserved.

New navigation sessions default to POSITION_THEN_SPIN with the same forward-only Regulated Pure Pursuit controller used by POSITION_ONLY. SmacPlanner2D uses the actual final path tangent during translation; a separate single Spin goal owns the endpoint heading after arrival. This avoids the former POSITION_THEN_SPIN fallback to DWB/Hybrid-Dubins terminal loops. Open-space paths favor shorter, smoother straight segments through lower traversal-cost weighting and bounded native Smac smoothing; occupied/unknown cells, full local footprint checks and keepout filters remain enforced. No BE/UI-only line simplification is used as a robot route. Nav2 may replan for obstacles and its real plan is relayed to BE/UI asynchronously.

Actual vendor YAML preflight: MOVE max 0.6 m/s forward, lateral 0, yaw max 1.2 rad/s; goal checker XY 0.06 m, verified arrival within 0.08 m. After arrival, at most one collision/zone-checked Spin requests endpoint yaw, with in-place angular commands at least 0.10 rad/s. The stationary arrival-pose observation window is 10 seconds and never publishes translation. The post-Spin heading policy below supersedes the former 0.10 rad second yaw check and second observation window. Mapping max speed remains 0.8 m/s and exploration retains holonomic motion.

Validation: 65 targeted offline localization/arrival/forward-profile/zone/path-relay tests passed in about 7 seconds; parameters were also generated against the actual installed Hiwonder base YAML and Nav2 libraries without starting ROS nodes or motor actions. No physical MOVE has been tested with this installed change. Next run: open a new map navigation session in the UI, use its generated session ID with zsh tools/navigation_session.zsh --session-id ID, initialize/verify the current real pose, then test a short MOVE. POSITION_ONLY remains an explicit opt-out via --arrival-mode POSITION_ONLY.

Primary references: Nav2 Humble SmacPlanner2D source https://api.nav2.org/nav2-humble/html/smac__planner__2d_8cpp_source.html ; Humble RPP source https://api.nav2.org/nav2-humble/html/regulated__pure__pursuit__controller_8cpp_source.html .

## 2026-11-02: finish each route step after its successful final turn

The 09:17 UTC run reported `spin completed successfully`, followed by the adapter's second `ENDPOINT_FINAL_POSE_NOT_CONFIRMED` check. A route stops when any MOVE fails, so later endpoints cannot start. This differs from the 09:08 run, where Spin itself aborted with `Collision Ahead - Exiting Spin`; that collision failure remains enforced.

At the operator's request, POSITION_THEN_SPIN now accepts a successful final Spin (or a heading already within 0.10 rad) as completion of endpoint orientation. It does not wait through a second AMCL stability window or reject the route merely because the post-turn map yaw differs by more than 0.10 rad. It still waits for wheels to stop and requires fresh localization. It sends no second corrective Spin. Every endpoint keeps its saved target heading, including intermediate route endpoints.

`verified_pose` always contains the measured pose. The optional existing `yaw_error_rad` field is included only within its contract bound of 0.15 rad; larger errors are recorded in `FINAL_ARRIVAL` diagnostics with the actual pose and position error. This is completion based on the Nav2 action, not a claim of exact heading accuracy. No BE schema change is needed. Thirty targeted arrival/profile/zone tests passed; no physical MOVE was dispatched for verification.

### Follow-up: consistent post-turn position acceptance

Request 8c9ad0a0-5563-479a-abc1-8cbbe3cc70e7 reached its first endpoint and completed Spin, but the measured post-turn position error was 0.0902989867 m, with yaw error only 0.06224147 rad. The former 0.08 m second position threshold rejected the step. Completed heading arrivals now use the same 0.10 m position envelope accepted before Spin; Nav2 still aims for its 0.06 m goal tolerance. The 0.12 m during-turn drift stop, fresh localization, collision and zone checks remain enforced. POSITION_ONLY retains its 0.08 m acceptance.

Pi handoff and both BE contract copies now accept `position_error_m` through 0.10 m, reporting the full measured value without clamping. Deploy BE before running new Pi results over the old 0.08 m contract. The follow-up passed 21 Pi tests and 40 BE result-contract cases. Neither verification dispatched motion.

## 2026-11-02: bounded session authorization during slow cloud reads

SessionPoller now asks the existing HTTPS session endpoint every 2 seconds instead of every second. A previously verified descriptor can survive a transient timeout/5xx for at most 15 seconds from its last successful verification, without renewing its timestamp on failures. TLS/auth/identity/closed-session failures invalidate immediately; session expiry still wins. This is authorization metadata, not a frozen robot pose. Live ROS scan, odometry, TF and localization freshness checks, local watchdog and 3-second MQTT heartbeats are unchanged. Initialization is reapplied only when its session revision changes, never simply because the next workflow MOVE starts.

Four transport cases and seven related session/identity cases passed without motor processes. An existing reactive-mapping yaw assertion in the broader session test file is still failing; it is outside this transport-only change. BE's companion route update reuses published shapes and avoids expiring its own admission heartbeat while its transaction holds the Robot lock. Runtime network URLs/CA are configured on Pi, not committed into this source.
