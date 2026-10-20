"""Generate manifest, simulated examples and exact field reference from installed assets."""
import asyncio
import json
import sys
import uuid
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from mediassist.contracts import Catalog, digest, read_json
from mediassist.store import Store
from mediassist.runtime import Runtime
from mediassist.adapters import SimAdapter

ROOT = Path(__file__).resolve().parents[1]


def save(path, value):
    (ROOT / path).write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def spec_text(spec):
    details = []
    for key in ("type", "const", "enum", "minimum", "exclusiveMinimum", "maximum", "minItems", "maxItems", "format"):
        if key in spec: details.append(key + "=" + json.dumps(spec[key], ensure_ascii=False))
    if "anyOf" in spec: details.append("anyOf=" + json.dumps(spec["anyOf"], ensure_ascii=False))
    return "; ".join(details) or "Xem schema/diễn giải bên dưới"


def fields(schema, prefix=""):
    rows = []
    for name, item in schema.get("properties", {}).items():
        path = prefix + name
        rows.append(f"| `{path}` | {'Có' if name in schema.get('required', []) else 'Theo nhánh/tùy chọn'} | {spec_text(item).replace('|', '&#124;')} |")
        if item.get("properties"): rows.extend(fields(item, path + "."))
        if item.get("items", {}).get("properties"): rows.extend(fields(item["items"], path + "[]."))
    return rows


async def main():
    catalog = Catalog(ROOT)
    manifest, examples = [], {}
    traces = []
    for (code, version), wf in catalog.workflows.items():
        manifest.append({"workflow_code": code, "version": version, "sha256": digest(wf), "capabilities": wf["required_capabilities"]})
        store = Store(":memory:")
        job = {"job_id": str(uuid.uuid4()), "workflow_code": code, "workflow_version": version,
               "workflow_hash": digest(wf), "input": read_json(ROOT / "examples" / (code + ".input.json"))}
        store.add_job(job)
        result = await Runtime(catalog, store, SimAdapter(), {"max_speed_mps": 0.05}).run(job)
        if result["status"] != "SUCCEEDED": raise RuntimeError(code + ": " + str(result))
        events = store.pending(10000)
        inputs = {}
        for e in events:
            d = e["data"]
            if e["event_type"] == "STEP_STARTED": inputs[d["job_step_id"]] = d
            if e["event_type"] == "STEP_RESULT" and d["outcome"] == "SUCCESS":
                start = inputs[d["job_step_id"]]
                examples.setdefault(start["type"], {"simulation": True, "input": start["resolved_input"], "output_data": d["result"]["data"]})
        if code == "WF_SECURE_MEDICAL_DELIVERY": traces = events
        store.close()
    save("catalog_manifest.json", manifest)
    save("examples/step_examples.json", examples)
    save("examples/delivery.simulated.events.json", traces)
    for name, example in examples.items(): save("examples/step_params/" + name.lower() + ".json", example["input"])
    lines = ["# 5. Hợp đồng STEP TYPE của runtime 0.5.1, workflow version 4", "",
        "Tạo từ schema và graph đang có trong bộ code. Các input/output mẫu bên dưới là mô phỏng, không phải số đo robot. Không dùng ID SIM_ hoặc grant mẫu để mở khóa thật.", "",
        "## Quy tắc đọc", "",
        "- Input là params đã resolve, không phải toàn bộ COMMAND. Output là result.data của STEP_RESULT.",
        "- Field đầu vào có additionalProperties=false không nhận thêm PWM/shell/code tùy ý.",
        "- MOVE: endpoint_id XOR nhóm endpoint_group_id + group_revision + candidate_endpoint_ids. max_speed_mps còn bị trần profile Pi giới hạn.",
        "- AUTHENTICATE: authenticated=true yêu cầu actor/grant/expiry không null. false có thể không biết các trường đó.",
        "- CONTROL_HARDWARE: UNLOCK thêm verification_id/purpose. Runtime kiểm grant trong cùng job; LOCK không tự chứng minh cửa đóng.",
        "- WAIT: DURATION cần duration_sec; EVENT cần event_type/correlation_key. Hai nhánh không có cùng cơ chế thực thi.",
        "- Các field con trong object tùy chọn chỉ bắt buộc khi object đó tồn tại. Điều kiện chi tiết nằm trong schemas/step_types.json.", ""]
    for kind, st in catalog.types.items():
        lines += ["## " + kind, "", f"Schema version: {st['schema_version']}. Executor: `{st['executor']}`.", "",
                  "### Input", "", "| Field | Bắt buộc | Kiểu/giới hạn |", "|---|---|---|", *fields(st["input_schema"]), "",
                  "### Output", "", "| Field | Bắt buộc | Kiểu/giới hạn |", "|---|---|---|", *fields(st["output_schema"]), ""]
        if kind == "EXECUTE_SENSING":
            lines += ["Metrics theo task_type: ROBOT_HEALTH cần sensors_ready/battery_raw/battery_percent(null); SIDE_CLEARANCE cần left_m/right_m/left_fresh/right_fresh; CROWD_DENSITY cần count>=0; RESTRICTED_AREA_INTRUSION cần intrusion_detected. health thực còn có age_sec và metadata chẩn đoán. captured_at là lúc tạo kết quả; age_sec của sample là từ lúc Pi nhận mẫu, không phải cam kết đồng hồ cảm biến độc lập.", ""]
        if kind == "CHECK_CONDITION":
            lines += ["Biểu thức SAFE_EXPR_V1: var lấy JSON pointer; eq/ne; gt/ge/lt/le; and/or/not; add. Không dùng chuỗi Python hoặc eval. SUCCESS với value=false là tính thành công nhưng điều kiện không đúng.", ""]
        if kind == "SET_CONTEXT":
            lines += ["assignments là map tên biến → biểu thức. Chỉ ghi vào vars; không ghi đè input/system/output/quyền. Biến phải là identifier, không bắt đầu bằng dấu gạch dưới.", ""]
        if kind in examples:
            lines += ["### Ví dụ mô phỏng", "", "```json", json.dumps(examples[kind], ensure_ascii=False, indent=2), "```", ""]
    lines += ["## 9 graph trong bộ code", "", "8 luồng nghiệp vụ: thông tin, dẫn đường bệnh nhân, giao nhận, đám đông, SOS, té ngã, khu hạn chế, endpoint/group. Khởi động là luồng kỹ thuật thứ9.", "",
              "Luồng giám sát và nhận biết hiện là các lần chạy có giới hạn. Bộ runtime không tự chạy detector té ngã hay tự lập lịch tuần tra nền; backend/perception service tạo job theo trigger. Crowd graph lặp tại checkpoint đầu vào, không tự đi mọi zone của bệnh viện. Không dùng một group đích thay thế như danh sách tuần tra bắt buộc.", ""]
    for wf in catalog.workflows.values():
        lines += ["### " + wf["code"], "", wf["name"], "", "Capabilities: " + ", ".join(wf["required_capabilities"]), "",
                  "Input ví dụ:", "", "```json", json.dumps(read_json(ROOT / "examples" / (wf["code"] + ".input.json")), ensure_ascii=False, indent=2), "```", "",
                  "| Mã bước | STEP TYPE | Params / dữ liệu truyền vào | Nhánh |", "|---|---|---|---|"]
        for s in wf["steps"]:
            branches = []
            for t in s["transitions"]:
                guard = " khi " + json.dumps(t["when"], ensure_ascii=False) if "when" in t else ""
                branches.append(t["on"] + guard + " → " + t.get("next", t.get("terminal", "")))
            lines.append("| " + s["code"] + " | " + s["type"] + " | `" + json.dumps(s["params"], ensure_ascii=False).replace("|", "&#124;") + "` | " + "; ".join(branches).replace("|", "&#124;") + " |")
        lines += ["", "Output tổng của workflow (binding):", "", "```json", json.dumps(wf["output_mapping"], ensure_ascii=False, indent=2), "```", ""]
    (ROOT / "docs/05_HOP_DONG_STEP_VA_WORKFLOW.md").write_text("\n".join(lines), encoding="utf-8")
    print("Generated manifest, 11 step examples and reference for", len(manifest), "workflows")


if __name__ == "__main__": asyncio.run(main())
