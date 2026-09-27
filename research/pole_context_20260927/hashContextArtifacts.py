"""Hash independent project evidence; never hash weekly archive reports."""
from pathlib import Path
import csv,hashlib
ROOT=Path(__file__).resolve().parents[2];P=Path(__file__).resolve().parent
files=[ROOT/x for x in ['config/pillarPoleValidationConfig.m','perception/offGroundFeatures/measurePillarPoleSupport.m','perception/offGroundFeatures/validatePillarPoleSupport.m','tests/pillarPoleValidationTest.m']]
files += [x for x in P.iterdir() if x.is_file() and x.suffix in {'.m','.py','.json','.csv','.md','.png','.pdf'} and x.name!='artifact_hashes.csv']
files += list((ROOT/'output/pole_context_20260927').glob('*.mat'))
with (P/'artifact_hashes.csv').open('w') as stream:
 w=csv.writer(stream,lineterminator='\n');w.writerow(['path','bytes','sha256'])
 for p in sorted(set(files)):
  h=hashlib.sha256()
  with p.open('rb') as f:
   for block in iter(lambda:f.read(1<<20),b''):h.update(block)
  w.writerow([str(p.relative_to(ROOT)),p.stat().st_size,h.hexdigest()])
