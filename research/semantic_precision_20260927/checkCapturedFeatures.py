"""Audit candidate totals, row widths and identities without changing inputs."""
from pathlib import Path
import csv,json,hashlib
ROOT=Path(__file__).resolve().parents[2];P=Path(__file__).resolve().parent;OUT=ROOT/'output/semantic_precision_20260927'
manifest=[]
for path in [OUT/f'{ds}_{semantic}_features.csv' for ds in ['mississippi','downtown'] for semantic in ['curb','trafficSign','facade'] if (OUT/f'{ds}_{semantic}_features.csv').exists()]:
 dataset,semantic=path.stem.split('_',1);semantic=semantic.removesuffix('_features')
 baseline={int(r['frame']):int(r['candidatePillarCount']) for r in csv.DictReader((P/f'{dataset}_baseline.csv').open()) if r['feature']==semantic}
 lines=path.read_text().splitlines();header=lines[0];width=len(header.split(','));actual={k:0 for k in baseline};seen=set()
 for line in lines[1:]:
  assert line and not line.startswith(','),(path,'Malformed or empty row')
  row=line.split(',');assert len(row)==width,(path,len(row),width)
  values=[float(x) for x in row];frame,pillar,count=values[:3]
  assert int(frame)==frame and int(pillar)==pillar and int(count)==count and count>=0
  key=(int(frame),int(pillar));assert key not in seen;seen.add(key);actual[int(frame)]+=1
 assert actual==baseline,(path,'Candidate rows differ from raw baseline counts')
 manifest.append(dict(file=str(path.relative_to(ROOT)),rows=len(lines)-1,columns=width,sha256=hashlib.sha256(path.read_bytes()).hexdigest(),perFrameCandidateCountsVerified=True))
(P/'feature_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
print(json.dumps(manifest,indent=2))
