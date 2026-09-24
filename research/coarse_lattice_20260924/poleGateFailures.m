function poleGateFailures()
s=load('output/coarse_lattice_20260924/pole_gate_table.mat'); T=s.T; B=T(T.isBase>0,:);
fprintf('baseline pole pillars in table: %d\n',height(B));
gates={'n<12',B.n<12;'h<1.5',B.h<1.5;'hStd<0.3',B.hStd<0.3;'tilt>20',B.tilt>20;'rStd>0.15',B.rStd>0.15;'rStd>0.22',B.rStd>0.22;'line>0.9',B.line>0.9;'context<=0.45',B.context<=0.45;'context==0',B.context==0;'point<0.6',B.point<0.6;'core==0',B.core==0};
for k=1:size(gates,1), fprintf('%-14s fails %4d (%.2f)\n',gates{k,1},nnz(gates{k,2}),mean(gates{k,2})); end
intrinsic=B.n>=12&B.h>=1.5&B.hStd>=0.3&B.tilt<=20&B.rStd<=0.22&B.line<=0.9;
fprintf('pass intrinsic(rStd0.22): %d; context>0.45: %d; point>=0.6: %d; both: %d\n',nnz(intrinsic),nnz(intrinsic&B.context>0.45),nnz(intrinsic&B.point>=0.6),nnz(intrinsic&B.context>0.45&B.point>=0.6));
N=T(T.isBase==0&T.n>=12&T.h>=1.5,:); q=[.1 .25 .5 .75 .9];
for v=["rStd","point","context","line","n","h","hStd","tilt"]
    fprintf('%-8s base %s | nonbase %s\n',v,mat2str(quantile(B.(v),q),3),mat2str(quantile(N.(v),q),3));
end
base=load('output/coarse_lattice_20260924/baseline_sequence.mat','records','geometry'); bf=cellfun(@(r) r.frame, base.records);
b=base.records{bf==1031}; ids=double(b.pillarIndices{b.semanticNames=="pole"}); [row,col]=ind2sub(base.geometry.mapSize,ids); c=base.geometry.origin+([col row]-0.5).*base.geometry.cellSize;
frame=loadPointCloudFrame('data/raw/MissisipiPointClouds.mat',1031); cfg=perceptionConfig(); cfg.coarseProbabilityCloud.storeDiagnostics=true;
p=perceiveFrame(frame,cfg); o=p.diagnostics.offGround; maps=o.columnMaps;
xyz=double([frame.x(:) frame.y(:) frame.z(:)]);
gseg=segmentGround(pillarizePointCloud(frame,cfg.voxel),cfg.groundSegmentation); gmask=false(numel(frame.x),1); gmask(gseg)=true;
fcfg=perceptionConfig("Mississippi","offline"); gsegF=segmentGround(pillarizePointCloud(frame,fcfg.voxel),fcfg.groundSegmentation); gmaskF=false(numel(frame.x),1); gmaskF(gsegF)=true;
for k=1:size(c,1)
  ob=floor((c(k,:)-maps.origin)./[maps.dx maps.dy])+1; inside=all(ob>=1)&&ob(1)<=maps.mapSize(2)&&ob(2)<=maps.mapSize(1);
  cnt=0; if inside, cnt=maps.pillarCounts(ob(2),ob(1)); end
  cell06=floor((c(k,:)-[-29.9 -29.9])/0.6); lo=-29.9+cell06*0.6; inCell=xyz(:,1)>=lo(1)&xyz(:,1)<lo(1)+0.6&xyz(:,2)>=lo(2)&xyz(:,2)<lo(2)+0.6&all(isfinite(xyz),2);
  fprintf('base pole (%.1f,%.1f) r=%.1f offground06=%d | cell pts %d, ground06 %d, ground03 %d, z [%.2f %.2f]\n',c(k,1),c(k,2),norm(c(k,:)),cnt,nnz(inCell),nnz(inCell&gmask),nnz(inCell&gmaskF),min(xyz(inCell,3)),max(xyz(inCell,3)));
end
end
