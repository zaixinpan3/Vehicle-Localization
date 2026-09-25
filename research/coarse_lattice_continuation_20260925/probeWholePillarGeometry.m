function T = probeWholePillarGeometry()
% probeWholePillarGeometry: Whole-0.6-m moment/phase descriptors for diagnosis.
% No 0.3 m detection runs here. Labels are read from the frozen study table.
% The output diagnoses information available to a coarse classifier; it is
% not a production model or an independent labelled ground-truth dataset.
    root=setupVehicleLocalization();
    cached=load(fullfile(root,'output','coarse_lattice_20260924','pole_feature_table2.mat'),'T');
    T=cached.T;
    cfg=perceptionConfig(); cfg.featureNames="pole";
    cfg.coarseProbabilityCloud.storeDiagnostics=true;
    names=["meanLocalX","meanLocalY","peakLocalX","peakLocalY", ...
        "minimumLocalX","minimumLocalY","maximumLocalX","maximumLocalY", ...
        "meanZ","minimumZ","maximumZ","cxx","cxy","cyy","cxz","cyz","czz", ...
        "maximumIntensity","maximumReflectivity"];
    values=nan(height(T),numel(names));
    for f=unique(T.frame).'
        frame=loadPointCloudFrame(fullfile(root,'data','raw','MissisipiPointClouds.mat'),f);
        p=perceiveFrame(frame,cfg);
        maps=p.diagnostics.offGround.columnMaps; stats=maps.statistics;
        [r,c]=ind2sub(maps.mapSize,double(stats.pillarIndices));
        lower=maps.origin+([c r]-1).*[maps.dx maps.dy];
        bins=round((lower-p.candidates.geometry.origin)./p.candidates.geometry.cellSize)+1;
        ids=sub2ind(p.candidates.geometry.mapSize,bins(:,2),bins(:,1));
        rows=find(T.frame==f); [found,k]=ismember(T.cell(rows),ids);
        assert(all(found),'Every cached candidate must resolve to a current whole pillar.');
        idx=double(stats.pillarIndices(k));
        values(rows,:)=[stats.meanXYZ(k,1:2)-lower(k,:), ...
            [maps.corePeakX(idx) maps.corePeakY(idx)]-lower(k,:), ...
            stats.minimumXYZ(k,1:2)-lower(k,:),stats.maximumXYZ(k,1:2)-lower(k,:), ...
            stats.meanXYZ(k,3),stats.minimumXYZ(k,3),stats.maximumXYZ(k,3), ...
            stats.covarianceXYZ(k,:),stats.intensity.maximum(k),stats.reflectivity.maximum(k)];
    end
    T=[T array2table(values,'VariableNames',cellstr(names))];
    writetable(T,fullfile(root,'output','coarse_lattice_20260924','whole_pillar_geometry.csv'));
end
