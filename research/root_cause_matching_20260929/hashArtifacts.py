"""Hash independent technical artifacts; never archive weekly-report hashes."""
from pathlib import Path
import hashlib,json
D=Path(__file__).resolve().parent;root=D.parent.parent
source_paths=['config/curbBoundaryGeometryConfig.m','config/perceptionConfig.m','perception/perceiveFrame.m',
 'perception/groundFeatures/estimateCurbBoundaryGeometry.m','perception/semanticProduct/applyCurbBoundaryScatter.m',
 'perception/offGroundFeatures/classifyPillarPoleSupport.m','localization/updateRobustPoseGraph.m',
 'tests/curbSurfaceGeometryTest.m','tests/scanDeskewTest.m','tests/robustPoseGraphTest.m']
files=[root/p for p in source_paths]
files.extend(p for p in D.rglob('*') if p.is_file() and '__pycache__' not in p.parts and p.name!='artifact_hashes.json')
files.extend(root/p for p in ['output/root_cause_matching_20260929/finalSurface_sources.mat',
 'output/root_cause_matching_20260929/finalSurface.mat','output/mississippi_mapping_calibrated/probability_cloud.mat',
 'config/mississippiLidarFrameCalibration.json'])
def digest(p):
 h=hashlib.sha256()
 with p.open('rb') as f:
  for block in iter(lambda:f.read(1024*1024),b''):h.update(block)
 return h.hexdigest()
manifest={'baselineCommit':'04c9f405f2ba3494d216ce690b2327ce54809244',
 'scope':'Independent project implementation and research outputs. Large local MAT inputs/outputs are referenced by hash, not published. No weekly or monthly report included.',
 'sha256':{str(p.relative_to(root)):digest(p) for p in sorted(set(files))}}
(D/'artifact_hashes.json').write_text(json.dumps(manifest,indent=2)+'\n')
print(f'Hashed {len(manifest["sha256"])} independent technical artifacts.')
