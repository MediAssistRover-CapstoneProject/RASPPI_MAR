"""Create a separate local servo test profile after operator inspection."""
import argparse
import copy
import json
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from mediassist.configuration import load_config, validate_config


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--base', type=Path, default=Path('config/site.local.json'))
    p.add_argument('--output', type=Path, default=Path('config/site.servo-test.json'))
    p.add_argument('--confirmed-id4-1500-2400', action='store_true', required=True)
    p.add_argument('--mechanism-inspected', action='store_true', required=True)
    a=p.parse_args()
    cfg=copy.deepcopy(load_config(a.base))
    cfg['servo']['commissioned']=True
    cfg['allowed_workflows']=[]
    cfg['motion']['commissioned']=False
    validate_config(cfg)
    a.output.parent.mkdir(parents=True, exist_ok=True)
    with a.output.open('x', encoding='utf-8') as f:
        json.dump(cfg,f,ensure_ascii=False,indent=2); f.write('\n')
    print('Saved', a.output, '; no servo command sent. Test each action explicitly; no physical feedback exists.')


if __name__ == '__main__': main()
