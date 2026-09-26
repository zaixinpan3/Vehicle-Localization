#!/usr/bin/env python3
"""Download pinned primary references into ignored output, retaining provenance."""
import hashlib,json,urllib.request
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2];DEST=Path(__file__).resolve().parent
OUT=ROOT/'output/mncav_multidrive_identification_20260925'
COMMIT='a8015bce561412ccf3711dd842ca84237f404081'
SOURCES={
 'dispatch.h':f'https://raw.githubusercontent.com/DataspeedInc-release/dbw_fca_ros-release/{COMMIT}/include/dbw_fca_can/dispatch.h',
 'DbwNode.cpp':f'https://raw.githubusercontent.com/DataspeedInc-release/dbw_fca_ros-release/{COMMIT}/src/DbwNode.cpp',
 'FCA_RU_FAQ.pdf':'https://www.dataspeedinc.com/app/uploads/2022/05/FCA_RU_FAQ.pdf',
 'ouster_manual.pdf':'https://data.ouster.io/downloads/software-user-manual/software-user-manual-v2.2.x.pdf'}
if __name__=='__main__':
 OUT.mkdir(parents=True,exist_ok=True)
 rows=[]
 for name,url in SOURCES.items():
  path=OUT/name
  try:
   if not path.exists():path.write_bytes(urllib.request.urlopen(url).read())
   rows.append(dict(file=str(path.relative_to(ROOT)),url=url,sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
  except urllib.error.HTTPError as error:
   rows.append(dict(url=url,downloadStatus=str(error),access='Read through browser PDF extraction; no local copy claimed'))
 (DEST/'sources.json').write_text(json.dumps(dict(rosReleaseCommit=COMMIT,sources=rows),indent=2)+'\n')
