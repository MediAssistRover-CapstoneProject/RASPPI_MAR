"""Check all 11 contracts and simulate installed workflows; no ROS/GPIO/network."""
import argparse
import asyncio
import json
from pathlib import Path
import sys
import uuid

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from mediassist.contracts import Catalog, read_json, digest
from mediassist.adapters import SimAdapter
from mediassist.runtime import Runtime
from mediassist.store import Store


async def check():
    catalog = Catalog(ROOT)
    examples = read_json(ROOT / 'examples/step_examples.json')
    if len(catalog.types) != 11 or set(catalog.types) != set(examples):
        raise ValueError('Expected exactly the 11 specified step types and examples')
    for kind, sample in examples.items():
        catalog.check_params(kind, sample['input'])
        catalog.check_output(kind, sample['output_data'])
    seen, jobs = set(), []
    for (code, version), wf in catalog.workflows.items():
        store = Store(':memory:')
        try:
            runtime = Runtime(catalog, store, SimAdapter(), {'max_speed_mps': .05})
            job = {'job_id': str(uuid.uuid4()), 'workflow_code': code, 'workflow_version': version,
                   'workflow_hash': digest(wf), 'input': read_json(ROOT / 'examples' / (code + '.input.json'))}
            store.add_job(job)
            result = await runtime.run(job)
            seen.update(e['data']['type'] for e in store.pending(10000) if e['event_type'] == 'STEP_STARTED')
            jobs.append({'workflow': code, 'status': result['status']})
        finally:
            store.close()
    report = {'mode': 'CONTRACT_AND_SIMULATION', 'hardware_tested': False, 'server_tested': False,
              'contract_types_passed': sorted(catalog.types), 'executed_types_simulated': sorted(seen),
              'workflows': jobs, 'missing_types': sorted(set(catalog.types) - seen)}
    report['passed'] = not report['missing_types'] and all(j['status'] == 'SUCCEEDED' for j in jobs)
    return report


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path)
    a=p.parse_args()
    report=asyncio.run(check())
    body=json.dumps(report, ensure_ascii=False, indent=2)
    print(body)
    if a.output:
        a.output.parent.mkdir(parents=True, exist_ok=True)
        a.output.write_text(body+'\n', encoding='utf-8')
    raise SystemExit(0 if report['passed'] else 1)


if __name__ == '__main__': main()
