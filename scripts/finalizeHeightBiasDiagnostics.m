function summary = finalizeHeightBiasDiagnostics(inputFolder, artifactFolder)
% finalizeHeightBiasDiagnostics: Verify/export numerical diagnostic evidence.
% Preserves datasets, caches, and existing figures. Creates one research plot.
    arguments
        inputFolder (1,1) string
        artifactFolder (1,1) string
    end
    if ~isfolder(artifactFolder), mkdir(artifactFolder); end
    baseline=load(fullfile(inputFolder,'diagnostic_results.mat'),'report');
    mechanism=load(fullfile(inputFolder,'mechanism_results.mat'),'report');
    cached=load(fullfile(inputFolder,'diagnostic_inputs.mat'),'inputs');
    b=baseline.report; m=mechanism.report;
    names=["registration","classes","heightScan","geometry","gradients", ...
        "rawResiduals","correctedResiduals","mechanisms","objectiveScan","classConflict"];
    for name=names
        copyfile(fullfile(inputFolder,name+".csv"),fullfile(artifactFolder,name+".csv"));
    end
    positive=b.registration(b.registration.map=="coarseSelf" & b.registration.variant=="xyzExactPose",:);
    gradientError=max(abs(b.gradients.analytic-b.gradients.finiteDifference));
    coordinateError=max(b.coordinateMaximumError);
    assert(all(positive.accepted) && max(positive.translationDifferenceM)<1e-5);
    assert(gradientError<5e-6 && coordinateError<1e-9);
    reference=readtable(fullfile(fileparts(inputFolder),'height_retention','registration_height.csv'),'TextType','string');
    reproductionError=0;
    for i=1:height(reference)
        row=reference(i,:);
        if row.heightOffsetMeters~=0 || row.tiltOffsetDegrees~=0, continue; end
        match=b.registration(b.registration.frame==row.frameIndex & b.registration.map=="fixed" & ...
            b.registration.source=="coarse" & b.registration.variant==row.mode & ...
            b.registration.start==row.initialPoseCase,:);
        assert(height(match)==1 && match.accepted==row.accepted && match.reason==row.reason);
        reproductionError=max(reproductionError,abs(match.translationDifferenceM-row.mapPoseTranslationDifferenceMeters));
    end
    assert(reproductionError<1e-6);
    for i=1:numel(cached.inputs)
        assert(~ismember(cached.inputs{i}.frame,cached.inputs{i}.observations.frameIndices));
    end
    file=fopen(fullfile(inputFolder,'original_ros_xyz.bin'),'r','ieee-le');
    assert(file>=0,'Run auditMississippiPointCloudTransform.py first.');
    cleanup=onCleanup(@()fclose(file));
    raw=fread(file,[3 inf],'double').';
    cfg=featureMapBuildConfig();
    root=fileparts(fileparts(mfilename('fullpath')));
    frame=loadPointCloudFrame(fullfile(root,'data',cfg.pointCloudMatPath),260);
    stored=double([reshape(frame.x.',[],1),reshape(frame.y.',[],1),reshape(frame.z.',[],1)]);
    keep=all(isfinite(raw),2)&all(isfinite(stored),2)&sum(raw.^2,2)>1&sum(raw.^2,2)<80^2;
    affine=[raw(keep,:),ones(nnz(keep),1)]\stored(keep,:);
    expected=[.931395 .364011 0;-.364011 .931395 0;0 0 1]* ...
        [.925216 -.367871 .093195;.368647 .929524 .009468;-.090110 .025595 .995625];
    extractorRms=sqrt(mean((stored(keep,:)-raw(keep,:)*expected.').^2,'all'));
    affineRms=sqrt(mean((stored(keep,:)-[raw(keep,:),ones(nnz(keep),1)]*affine).^2,'all'));
    assert(extractorRms<2e-6);
    clear cleanup;
    summary=struct('baselineCommit',"2fc2ba3d2c882f4a59edf506cae85cd329344432", ...
        'matlabVersion',string(version),'registrationRuns',height(b.registration)+height(m.mechanisms), ...
        'heightScoreSamples',height(b.heightScan),'longitudinalScoreSamples',height(m.objectiveScan), ...
        'maximumGradientAbsoluteError',gradientError,'coordinateMaximumErrorM',coordinateError, ...
        'maximumPositiveControlTranslationErrorM',max(positive.translationDifferenceM), ...
        'maximumBaselineReproductionErrorM',reproductionError, ...
        'diagnosticPitchCorrectionDegrees',m.pitchCorrectionDegrees, ...
        'pitchCalibrationFrames',[260 261 262 263 264 265 266], ...
        'pitchCalibrationClass',"curb",'originalBagExtractorRmsM',extractorRms, ...
        'originalBagAffineFitRmsM',affineRms,'fittedStoredFromRosAffineTranspose',affine, ...
        'pointMask',"finite raw range 1 to 80 m",'auditPointCount',nnz(keep), ...
        'interpretation',"Recorded-map-pose consistency and geometry diagnostics; no independent truth.");
    writeText(fullfile(artifactFolder,'validation.json'),jsonencode(summary,PrettyPrint=true));
    copyfile(fullfile(inputFolder,'original_ros_metadata.json'),fullfile(artifactFolder,'original_ros_metadata.json'));
    fig=figure('Name','D2D bias mechanisms: height drift and sampling density', ...
        'Color','w','Position',[40 80 1480 840]);
    layout=tiledlayout(fig,2,3,'TileSpacing','compact','Padding','compact');
    residualClasses=unique(string(m.rawResiduals.class),'stable');
    objectiveClasses=unique(string(m.objectiveScan.class),'stable');
    colors=lines(max(numel(residualClasses),numel(objectiveClasses)));
    frames=[260 550 900];
    for i=1:3
        ax=nexttile(layout,i); hold(ax,'on');
        for classIndex=1:numel(residualClasses)
            name=residualClasses(classIndex);
            raw=m.rawResiduals(m.rawResiduals.frame==frames(i)&m.rawResiduals.class==name,:);
            cor=m.correctedResiduals(m.correctedResiduals.frame==frames(i)&m.correctedResiduals.class==name,:);
            plot(ax,raw.travelM,raw.medianZResidualM,'-o','Color',colors(classIndex,:), ...
                'LineWidth',1.5,'DisplayName',name+" original");
            plot(ax,cor.travelM,cor.medianZResidualM,'--s','Color',colors(classIndex,:), ...
                'LineWidth',1.5,'DisplayName',name+" corrected");
        end
        yline(ax,0,':','HandleVisibility','off'); grid(ax,'on');
        xlabel(ax,'Travel from query frame (m)'); ylabel(ax,'Median map Z - query Z (m)');
        title(ax,sprintf('Frame %d: raw same-class pairs',frames(i)));
        ylim(ax,[-.06 .57]);
        if i==1, legend(ax,'Location','northwest','FontSize',9); end
        ax=nexttile(layout,i+3); hold(ax,'on');
        classNames=objectiveClasses;
        for j=1:numel(classNames)
            t=m.objectiveScan(m.objectiveScan.frame==frames(i)&m.objectiveScan.mode=="xy"& ...
                m.objectiveScan.class==classNames(j),:);
            plot(ax,t.longitudinalOffsetM,t.score,'Color',colors(j,:), ...
                'LineWidth',1.5+(classNames(j)=="all"),'DisplayName',classNames(j));
        end
        xline(ax,0,'--','Recorded pose','HandleVisibility','off'); grid(ax,'on');
        xlabel(ax,'Longitudinal offset (m), fixed recorded yaw'); ylabel(ax,'Normalized XY overlap');
        title(ax,sprintf('Frame %d: six future map frames',frames(i)));
        if i==1, legend(ax,'Location','northwest','FontSize',9); end
    end
    title(layout,sprintf('One %.3f deg pitch diagnostic fitted on frame 260 curb; frame 550/900 not used in fitting',m.pitchCorrectionDegrees), ...
        'Color',[.1 .1 .1]);
    % Desktop dark-theme defaults must not make exported labels unreadable.
    set(findall(fig,'Type','axes'),'Color','w','XColor',[.15 .15 .15], ...
        'YColor',[.15 .15 .15],'GridColor',[.5 .5 .5]);
    set(findall(fig,'Type','text'),'Color',[.1 .1 .1]);
    set(findall(fig,'Type','legend'),'Color','w','TextColor',[.1 .1 .1]);
    set(findall(fig,'Type','constantline'),'Color',[.35 .35 .35]);
    exportgraphics(fig,fullfile(artifactFolder,'bias_mechanisms.png'),'Resolution',160);
    exportgraphics(fig,fullfile(artifactFolder,'bias_mechanisms.pdf'),'ContentType','vector');
    exportgraphics(fig,fullfile(artifactFolder,'bias_mechanisms.svg'),'ContentType','vector');
    svgPath=fullfile(artifactFolder,'bias_mechanisms.svg');
    svgText=regexprep(fileread(svgPath),'[ \t]+(?=\r?\n)','');
    writeText(svgPath,strtrim(svgText));
    disp(summary);
end

function writeText(path,content)
    file=fopen(path,'w'); assert(file>=0);
    cleanup=onCleanup(@()fclose(file));
    fprintf(file,'%s\n',content);
end
