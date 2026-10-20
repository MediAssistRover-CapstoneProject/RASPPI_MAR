"""Run relevant tests and static checks; write an honest reproducible report."""
import ast
import json
import re
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from mediassist.contracts import Catalog, digest
from mediassist.configuration import load_config


def main():
    json_files = [p for p in ROOT.rglob("*.json") if "state" not in p.parts and p.name != "validation_report.json"]
    for path in json_files: json.loads(path.read_text(encoding="utf-8"))
    python_files = [p for p in ROOT.rglob("*.py") if "build" not in p.parts]
    for path in python_files: ast.parse(path.read_text(encoding="utf-8"), filename=str(path), feature_version=(3, 10))
    ET.parse(ROOT / "config/robot_addons.xacro")
    ET.parse(ROOT / "config/navigate_no_recovery.xml")
    load_config(ROOT / "config/site.local.json")
    load_config(ROOT / "config/site.example.json")
    import yaml
    yaml.safe_load((ROOT / "rviz/mediassist.rviz").read_text(encoding="utf-8"))
    for doc in [ROOT / "README.md", *(ROOT / "docs").glob("*.md")]:
        content = doc.read_text(encoding="utf-8")
        for code in re.findall(r"```json\s*\n(.*?)\n```", content, re.S): json.loads(code)
        for target in re.findall(r"\]\(([^)]+)\)", content):
            if "://" not in target and not target.startswith("#"):
                destination = doc.parent / target.split("#")[0]
                if destination.resolve() == (ROOT / "validation_report.json").resolve(): continue
                if not destination.exists(): raise RuntimeError(f"Broken link: {doc.name} -> {target}")
    catalog = Catalog(ROOT)
    manifest = json.loads((ROOT / "catalog_manifest.json").read_text(encoding="utf-8"))
    for row in manifest:
        if row["sha256"] != digest(catalog.get(row["workflow_code"], row["version"])): raise RuntimeError("Stale manifest")
    for kind, example in json.loads((ROOT / "examples/step_examples.json").read_text(encoding="utf-8")).items():
        catalog.check_params(kind, example["input"])
        catalog.check_output(kind, example["output_data"])
    result = subprocess.run([sys.executable, "-X", "utf8", "-m", "unittest", "discover", "-s", "tests", "-v"], cwd=ROOT, capture_output=True, text=True, encoding="utf-8")
    output = result.stdout + result.stderr
    (ROOT / "validation_test_output.txt").write_text(output, encoding="utf-8")
    count = re.search(r"Ran (\d+) tests", output)
    report = {"generated_at": datetime.now(timezone.utc).isoformat(), "python": sys.version.split()[0],
              "host": sys.platform, "python_310_syntax_files": len(python_files), "valid_json_files": len(json_files),
              "step_types": len(catalog.types), "workflows": len(catalog.workflows), "unit_and_loopback_tests": int(count.group(1)) if count else None,
              "test_exit_code": result.returncode, "target_pi_tested": False, "physical_robot_tested": False,
              "notes": ["ROS/GPIO/PWM/Nav2 adapters source-reviewed and syntax checked; target acceptance pending", "Live services AI/identity/interaction/notification need backend implementation", "No credentials included in bundle"]}
    (ROOT / "validation_report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    if result.returncode: print(output); raise SystemExit(result.returncode)


if __name__ == "__main__": main()
