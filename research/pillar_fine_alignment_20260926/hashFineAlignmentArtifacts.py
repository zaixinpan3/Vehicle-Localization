"""Identify independent technical inputs and outputs; never hash archive reports."""
from pathlib import Path
import csv
import hashlib

ROOT = Path(__file__).resolve().parents[2]
FOLDER = Path(__file__).resolve().parent
PATHS = [
    ROOT/'config/structuralPillarConfig.m', ROOT/'config/pillarPoleValidationConfig.m',
    ROOT/'perception/offGroundFeatures/analyzeStructuralPillars.m',
    ROOT/'perception/offGroundFeatures/detectPolePillars.m',
    ROOT/'perception/offGroundFeatures/measurePillarPoleSupport.m',
    ROOT/'perception/offGroundFeatures/validatePillarPoleSupport.m',
    ROOT/'tests/pillarPoleValidationTest.m', ROOT/'tests/pillarFineAlignmentTest.m',
]
PATHS += [p for p in FOLDER.rglob('*') if p.is_file() and p.suffix in {'.m','.py','.json','.csv','.png','.pdf','.md'} and p.name != 'artifact_hashes.csv']
PATHS += list((ROOT/'output/pillar_fine_alignment_20260926').glob('*.mat'))
with (FOLDER/'artifact_hashes.csv').open('w') as stream:
    writer = csv.writer(stream, lineterminator="\n")
    writer.writerow(['path','bytes','sha256'])
    for p in sorted(set(PATHS)):
        h=hashlib.sha256()
        with p.open('rb') as f:
            for block in iter(lambda:f.read(1<<20),b''):
                h.update(block)
        writer.writerow([str(p.relative_to(ROOT)),p.stat().st_size,h.hexdigest()])
