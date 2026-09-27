function probeGeometryControls(prefix)
% probeGeometryControls: Exercise physical structures before profile promotion.
    if nargin<1,prefix='';end
    cfg=geometryCandidateConfig('Mississippi',prefix);gridCfg=cfg.voxel;gridCfg.exclusionHalfSize=0;
    cloud=coarseSemanticProbabilityCloudConfig(gridCfg);cloud.semanticNames="pole";
    z=repelem(linspace(-1,3,12).',2);p=[repmat([1.29;1.31],12,1),ones(size(z))*1.05,z];
    g=pillarizePointCloud(p,gridCfg);r=analyzeStructuralPillars(g,cfg.offGroundFeatures,cloud);
    fprintf('Split tight shaft: %d owners\n',nnz(r.poleCellMask));assert(nnz(r.poleCellMask)==2);
    z=linspace(-1,3,60).';a=(1:60).'*2.399;p=[10.12+.025*cos(a),2.16+.025*sin(a),z];
    rng(14);clutter=[10.04+.52*rand(800,1),1.9+.52*rand(800,1),3.6+.5*rand(800,1)];
    g=pillarizePointCloud([p;clutter],gridCfg);r=analyzeStructuralPillars(g,cfg.offGroundFeatures,cloud);
    fprintf('Minority shaft below clutter: %d owners\n',nnz(r.poleCellMask));assert(nnz(r.poleCellMask)>=1);
    [x,z]=meshgrid(8:.04:12,-1:.08:3);p=[x(:),ones(numel(x),1)*2,z(:)];
    g=pillarizePointCloud(p,gridCfg);r=analyzeStructuralPillars(g,cfg.offGroundFeatures,cloud);
    fprintf('Wide wall: %d owners\n',nnz(r.poleCellMask));assert(~any(r.poleCellMask,'all'));
    z=[(0:.05:.5)';(3:.05:3.5)'];p=[10.3+.01*cos(z*20),2.3+.01*sin(z*20),z];
    g=pillarizePointCloud(p,gridCfg);r=analyzeStructuralPillars(g,cfg.offGroundFeatures,cloud);
    fprintf('Separated blobs: %d owners\n',nnz(r.poleCellMask));assert(~any(r.poleCellMask,'all'));
end
