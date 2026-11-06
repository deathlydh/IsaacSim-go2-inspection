"""python tools/validate_dataset.py data/gauges/<id>"""
import sys,json
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from inspection.dataset import validate
if __name__=='__main__':
    result=validate(sys.argv[1]);print(json.dumps(result,indent=2));raise SystemExit(0 if result['valid'] else 1)
