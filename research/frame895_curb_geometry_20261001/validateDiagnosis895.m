function validateDiagnosis895()
% validateDiagnosis895 Verify saved causal controls and diagnostic invariants.
    setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));out='output/frame895_curb_geometry_20261001';
    a=load(fullfile(out,'audit.mat'));b=load(fullfile(out,'fallback_map.mat'));f=load(fullfile(out,'factors.mat'));m=load(fullfile(out,'map_shape.mat'));
    assert(all(a.coverage.reproductionMaxM==0));assert(isequaln(a.pools{1,1},a.d.source));
    baseline=a.d.summary.positionErrorM;
    assert(abs(a.controls.errorM(1)-baseline)<1e-8);
    assert(abs(b.controls.errorM(1)-baseline)<1e-8);
    assert(abs(m.controls.errorM(1)-baseline)<1e-8);
    assert(all(f.controls.exitFlag==1)&&all(m.controls.accepted));
    c=b.component;assert(norm(c.covariance-c.withinBlockCovariance-c.stableOffsetCovariance-c.referenceMeanCovariance,'fro')<1e-12);
    assert(norm(a.d.fixed.components.covariance(:,:,685)-c.covariance,'fro')<1e-12);
    gates=cell(height(b.rejected),6);
    for j=1:height(b.rejected)
        row=b.rejected(j,:);p=a.perceptions{row.frame-890};g=p.diagnostics.ground;cloud=p.probabilityCloud;id=row.pillar;
        proposed=[g.moments.mean(id,:)+[row.dx,row.dy],g.moments.meanZ(id)];
        raw=(proposed-cloud.projectionTranslation)*cloud.projectionRotation;
        bin=floor((raw(1:2)-g.cellOrigin)./g.cellSize)+1;[r,col]=ind2sub(g.cellMapSize,id);
        gates(j,:)={row.variant,row.frame,id,bin(1)==col&&bin(2)==r,bin(1)-col,bin(2)-r};
    end
    gates=cell2table(gates,VariableNames={'variant','frame','pillar','insideOriginalPillar','deltaColumn','deltaRow'});
    writetable(gates,fullfile(dest,'rejected_boundary_gates.csv'));
    files=dir(fullfile(dest,'*.m'));counts=zeros(numel(files),1);
    for k=1:numel(files),counts(k)=numel(checkcode(fullfile(files(k).folder,files(k).name),'-struct','-config=factory'));end
    assert(all(counts==0));
    summary=struct('matlabVersion',version,'baselineErrorM',baseline,'freshFramesReproducedExactly',a.coverage.frame.', ...
        'sourceWindowReproducedExactly',true,'publishedCovarianceDecompositionVerified',true, ...
        'frozenFactorOptimizationsConverged',height(f.controls),'productionShapeControlsAccepted',height(m.controls), ...
        'angleOnlyPositionErrorM',f.controls.errorM(f.controls.variant=="within_block_target_angle"), ...
        'allCurbWithinShapeErrorM',m.controls.errorM(5),'allCurbWithinShapeReferenceTransportErrorM',m.controls.errorM(6), ...
        'factoryAnalyzedFiles',{string({files.name})},'factoryAnalyzerFindings',counts, ...
        'productionSourceChanged',false,'fullRouteRerun',false,'desktopWindowOpened',false);
    fid=fopen(fullfile(dest,'validation.json'),'w');fprintf(fid,'%s\n',jsonencode(summary,PrettyPrint=true));fclose(fid);disp(summary);disp(gates);
end
