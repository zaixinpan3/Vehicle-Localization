"""Generate a local diagnostic copy of the fine validator with gate observations.

Assignments are inserted around unchanged decisions; this copy is never used
by online perception or to regenerate reference labels.
"""
from pathlib import Path
import hashlib
import json

ROOT = Path(__file__).resolve().parents[2]
source = (ROOT / "perception/refinePerceptionCandidates.m").read_text()
start = source.index("function accepted = validatePolePoints(")
end = source.index("\nfunction [context,candidates]=prepareFineStructuralCandidates", start)
body = source[start:end].replace("function accepted = validatePolePoints(",
                                 "function [accepted,details] = traceFinePolePoints(", 1)
body = body.replace("    accepted = false(size(points, 1), 1);",
                    "    details=struct([]);\n    accepted = false(size(points, 1), 1);", 1)
insertions = {
    "        p = points(rows, :);": """
        entry=numel(details)+1;
        entryData=struct('stage',"count_height",'x',mean(p(:,1)),'y',mean(p(:,2)), ...
            'count',size(p,1),'height',max(p(:,3))-min(p(:,3)), ...
            'supportedHeight',NaN,'supportedCount',NaN,'meanRatio',NaN,'massRatio',NaN, ...
            'tilt',NaN,'radialRms',NaN,'maximumStd',NaN,'aspect',NaN, ...
            'longestRun',NaN,'isolation',NaN,'contrast',NaN,'accepted',0);
        if isempty(details),details=entryData;else,details(entry)=entryData;end
""",
    "        qualified = objectCounts >= maps.occupiedLayerMinPoints & ratio > cfg.poleMinimumSliceRatio;": """
        details(entry).stage="slice_support";
        details(entry).supportedHeight=nnz(qualified)*geometry.voxelSize(3);
        details(entry).meanRatio=mean(ratio(qualified));
        details(entry).massRatio=sum(objectCounts(qualified))/max(sum(neighborCounts(qualified)),1);
""",
    "        supported = qualified(zBin);": """
        details(entry).stage="supported_count";details(entry).supportedCount=nnz(supported);
""",
    "        coefficients = design(supported,:) \\ p(supported, 1:2);": """
        details(entry).stage="tilt";details(entry).tilt=atand(norm(coefficients(2,:)));
""",
    "        spread=sort(eig(cov(transverse)));": """
        details(entry).stage="wide_surface";details(entry).radialRms=radialRms;
        details(entry).maximumStd=sqrt(spread(2));details(entry).aspect=sqrt(spread(2)/max(spread(1),eps));
""",
    "            runLength=find(edges==-1)-find(edges==1);": """
            details(entry).stage="short_continuity_tilt";
            details(entry).longestRun=max(runLength)*geometry.voxelSize(3);
""",
    "        if shortSupport && radialRms > cfg.poleShortSupportMaximumRadialRms": """
            details(entry).stage="short_radial";
""",
    "            contrast=separation.coreCount/max(separation.neighborhoodCount-separation.coreCount,1)*areaRatio;": """
            details(entry).stage="isolation";details(entry).isolation=separation.coreFraction;
            details(entry).contrast=contrast;
""",
    "        accepted(rows)=keep;": """
        details(entry).stage="final_support";details(entry).accepted=nnz(keep);
""",
}
for anchor, observation in insertions.items():
    assert body.count(anchor) == 1, anchor
    body = body.replace(anchor, anchor + observation, 1)
output = ROOT / "output/pillar_fine_alignment_20260926/diagnostic"
output.mkdir(parents=True, exist_ok=True)
path = output / "traceFinePolePoints.m"
path.write_text(body)
metadata = {"source": "perception/refinePerceptionCandidates.m",
            "sourceSha256": hashlib.sha256(source.encode()).hexdigest(),
            "generated": str(path.relative_to(ROOT)),
            "generatedSha256": hashlib.sha256(body.encode()).hexdigest(),
            "purpose": "Observe unchanged gate decisions; not a reference or production detector"}
(output / "trace_source.json").write_text(json.dumps(metadata, indent=2) + "\n")
print(json.dumps(metadata, indent=2))
