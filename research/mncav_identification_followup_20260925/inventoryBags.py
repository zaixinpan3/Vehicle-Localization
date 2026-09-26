#!/usr/bin/env python3
"""Inventory topic metadata in local bags without decoding the recordings."""
import csv
import json
from pathlib import Path
from rosbags.rosbag1 import Reader

ROOT=Path(__file__).resolve().parents[2]
DEST=Path(__file__).resolve().parent


def main():
    rows=[];schemas={}
    for path in sorted((ROOT/'data/raw').rglob('*.bag')):
        with Reader(path) as reader:
            topics={c.topic:c.msgcount for c in reader.connections}
            relevant={k:v for k,v in topics.items() if k.startswith('/vehicle/') and any(
                word in k for word in ['report','imu','can_rx'])}
            for c in reader.connections:
                if c.topic in relevant and c.msgtype not in schemas:
                    schemas[c.msgtype]=c.msgdef.data
            rows.append(dict(bag=str(path.relative_to(ROOT)),bytes=path.stat().st_size,
                rosDurationSeconds=(reader.end_time-reader.start_time)/1e9,topicCount=len(topics),
                inspvaMessages=topics.get('/novatel/oem7/inspva',0),
                relevantTopics=json.dumps(relevant,sort_keys=True)))
    with (DEST/'bag_inventory.csv').open('w') as handle:
        writer=csv.DictWriter(handle,fieldnames=list(rows[0]),lineterminator='\n')
        writer.writeheader();writer.writerows(rows)
    out=ROOT/'output/mncav_identification_followup_20260925'
    (out/'recorded_message_schemas.json').write_text(json.dumps(schemas,indent=2)+'\n')
    print('Inventoried',len(rows),'bags and',len(schemas),'relevant message definitions.')


if __name__=='__main__':main()
