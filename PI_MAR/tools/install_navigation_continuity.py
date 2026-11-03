"""Install staged source after stopping stack; never overwrite secrets/configuration."""
import argparse,ast,hashlib,json,os,shutil,sys
from datetime import datetime,timezone
from pathlib import Path

def digest(path):
    return hashlib.sha256(path.read_bytes().replace(b'\r\n',b'\n')).hexdigest() if path.is_file() else None

def install(stage,root,apply=False):
    stage=stage.resolve(strict=True);root=root.resolve(strict=True)
    manifest=json.loads((stage/'manifest.json').read_text(encoding='utf-8'))
    pending=[]
    for name,record in manifest.items():
        relative=Path(name)
        if relative.is_absolute() or '..' in relative.parts or not name.startswith(('pi/mediassist/','tools/','handoff/')):
            raise RuntimeError('INVALID_MANIFEST_PATH')
        source=stage/'files'/relative;destination=root/relative
        if not source.resolve().is_relative_to(stage) or not destination.resolve().is_relative_to(root):
            raise RuntimeError('PATH_OUTSIDE_PROJECT')
        if digest(source)!=record['after']:raise RuntimeError('PATCH_DIGEST_MISMATCH: '+name)
        if source.suffix=='.py':ast.parse(source.read_text(encoding='utf-8'),filename=name)
        current=digest(destination)
        if current==record['after']:continue
        if current not in record['accepted']:raise RuntimeError('SOURCE_CHANGED: '+name+'; no files replaced')
        pending.append((name,source,destination,current))
    print('FILES_TO_UPDATE',len(pending))
    if not apply or not pending:return
    # Recheck while owning the same lock used by robot_stack/configuration.
    sys.path[:0]=[str(root/'pi'),str(root/'tools')]
    from configure_mapping import stopped_configuration
    with stopped_configuration(root/'pi/config/mqtt.worker.json'):
        for name,source,destination,current in pending:
            if digest(destination)!=current:raise RuntimeError('SOURCE_CHANGED_DURING_CHECK: '+name)
        stamp=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
        backup=root/'pi/state/patch-backups'/('navigation-continuity-'+stamp);backup.mkdir(parents=True)
        old={};replaced=[]
        try:
            for name,source,destination,current in pending:
                old[name]=current
                if current is not None:
                    saved=backup/name;saved.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(destination,saved)
            (backup/'before.json').write_text(json.dumps(old,indent=2),encoding='utf-8')
            for name,source,destination,current in pending:
                destination.parent.mkdir(parents=True,exist_ok=True)
                temporary=destination.with_suffix(destination.suffix+'.continuity-new')
                try:
                    temporary.write_bytes(source.read_bytes())
                    temporary.chmod(destination.stat().st_mode&0o777 if current else 0o644)
                    os.replace(temporary,destination);replaced.append((name,destination,current))
                finally:temporary.unlink(missing_ok=True)
        except BaseException:
            for name,destination,current in reversed(replaced):
                if current is None:destination.unlink(missing_ok=True)
                else:shutil.copy2(backup/name,destination)
            raise
    print('SOURCE_INSTALLED; check pi/mediassist/step_contract.py VERSION')
    print('BACKUP',backup)
    print('No robot started; IP/TLS/CA, profiles and database identities unchanged')

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--stage',type=Path,default=Path(__file__).resolve().parent)
    p.add_argument('--root',type=Path,default=Path('/home/ubuntu/mediassist-mqtt-2026-10-08'))
    p.add_argument('--apply',action='store_true');args=p.parse_args()
    install(args.stage,args.root,args.apply)
if __name__=='__main__':main()
