"""Run isolated offline suites; none of these scripts accesses the network."""
import pathlib,subprocess,sys
ROOT=pathlib.Path(__file__).resolve().parent.parent
(ROOT/'tests/results/tmp').mkdir(parents=True,exist_ok=True)
for name in ('test_recovery.py','run_core.py','test_lan.py','test_installer.py'):
    result=subprocess.run([sys.executable,str(ROOT/'tests'/name)],cwd=ROOT)
    if result.returncode:
        raise SystemExit(result.returncode)
print('All four offline suites passed (44 + 35 + 19 + 5).')
