function summary = finalizeGeometricRegistrationStudy(outputFolder, exportFolder)
% finalizeGeometricRegistrationStudy: Export actual measurements and a figure.
% Requires completed evaluateGeometricRegistration runs and the separately
% fitted calibration. Exported files contain no raw route coordinates.
    arguments
        outputFolder (1,1) string
        exportFolder (1,1) string
    end
    if ~isfolder(exportFolder), mkdir(exportFolder); end
    original=load(fullfile(outputFolder,'evaluation_identity.mat'),'report');
    fit=load(fullfile(outputFolder,'pitch_calibration.mat'),'vlGeoCalibration','vlGeoCalibrationFit');
    tag=string(matlab.lang.makeValidName(fit.vlGeoCalibration.identifier));
    calibrated=load(fullfile(outputFolder,'evaluation_'+tag+'.mat'),'report');
    metrics=[original.report.metrics;calibrated.report.metrics];
    metrics(:,{'x','y','yaw'})=[];
    timing=[original.report.timing;calibrated.report.timing];
    writetable(metrics,fullfile(exportFolder,'registration.csv'));
    writetable(timing,fullfile(exportFolder,'timing.csv'));
    writetable(fit.vlGeoCalibrationFit.pairs,fullfile(exportFolder,'calibration_training.csv'));
    fidelity=readtable(fullfile(outputFolder,'perception_fidelity_timing.csv'));
    writetable(fidelity,fullfile(exportFolder,'perception_fidelity_timing.csv'));
    tests=load(fullfile(outputFolder,'tests.mat'),'vlGeoAllTests');
    analysis=load(fullfile(outputFolder,'code_analysis.mat'),'vlGeoCheckFiles','vlGeoCheck');
    testTable=table(tests.vlGeoAllTests); testTable.Details=[];
    writetable(testTable,fullfile(exportFolder,'tests.csv'));
    analyzerTable=table(analysis.vlGeoCheckFiles(:),cellfun(@numel,analysis.vlGeoCheck), ...
        'VariableNames',{'file','findings'});
    writetable(analyzerTable,fullfile(exportFolder,'code_analyzer.csv'));
    summary=struct('interpretation',"Recorded-pose consistency, not independent ground truth", ...
        'matlabVersion',version, ...
        'onlineState',"X,Y,psi",'frames',original.report.frames,'starts',original.report.starts, ...
        'registrationRuns',height(metrics),'testsPassed',nnz([tests.vlGeoAllTests.Passed]), ...
        'testsFailed',nnz([tests.vlGeoAllTests.Failed]),'testsIncomplete',nnz([tests.vlGeoAllTests.Incomplete]), ...
        'codeAnalysisFiles',numel(analysis.vlGeoCheckFiles),'codeAnalysisFindings',sum(cellfun(@numel,analysis.vlGeoCheck)), ...
        'calibration',fit.vlGeoCalibration,'calibrationFit',rmfield(fit.vlGeoCalibrationFit,'pairs'), ...
        'currentMedianCoarseMs',median(fidelity.currentCoarseMs), ...
        'previousMedianCoarseMs',median(fidelity.previousCoarseMs));
    groups=cell(0,9);
    for calibration=unique(metrics.calibration).'
        for variant=unique(metrics.variant).'
            rows=metrics.calibration==calibration & metrics.variant==variant;
            accepted=rows & metrics.accepted;
            elapsed=timing.registrationMs(timing.calibration==calibration & timing.variant==variant);
            groups(end+1,:)={calibration,variant,nnz(rows),nnz(accepted), ...
                nnz(accepted & metrics.translationDifferenceM>1),median(metrics.translationDifferenceM(accepted)), ...
                max(metrics.translationDifferenceM(accepted)),max(abs(metrics.yawDifferenceDeg(accepted))),median(elapsed)}; %#ok<AGROW>
        end
    end
    summary.groups=cell2table(groups,'VariableNames',{'calibration','variant','runs','accepted', ...
        'acceptedOverOneMeter','medianAcceptedDifferenceM','maximumAcceptedDifferenceM', ...
        'maximumAcceptedYawDifferenceDeg','medianRegistrationMs'});
    writetable(summary.groups,fullfile(exportFolder,'summary.csv'));
    fid=fopen(fullfile(exportFolder,'summary.json'),'w'); assert(fid>=0);
    closeFile=onCleanup(@() fclose(fid));
    fwrite(fid,jsonencode(summary,'PrettyPrint',true)); clear closeFile
    figureHandle=figure('Visible','off','Color','w','Theme','light','Position',[100 100 1200 520], ...
        'DefaultAxesFontSize',10,'DefaultTextFontSize',10);
    closeFigure=onCleanup(@() close(figureHandle));
    layout=tiledlayout(figureHandle,1,2,'TileSpacing','compact');
    variants=["densityXY","geometricXY","geometricHeight"];
    labels=["Legacy density","Gaussian geometry XY","Gaussian geometry + height"];
    colors=[.05 .35 .70;.85 .30 .05;.15 .55 .30];
    for panel=1:2
        ax=nexttile(layout); hold(ax,'on'); handles=gobjects(3,1);
        calibration="identity";
        if panel==2, calibration=tag; end
        for v=1:3
            rows=metrics.calibration==calibration & metrics.variant==variants(v) & metrics.start==1;
            data=sortrows(metrics(rows,:),'frame'); x=(1:height(data))+.18*(v-2);
            handles(v)=plot(ax,x,data.translationDifferenceM,'o-','Color',colors(v,:),'LineWidth',1.1,'MarkerSize',5);
            rejected=~data.accepted;
            plot(ax,x(rejected),data.translationDifferenceM(rejected),'x','Color',colors(v,:),'LineWidth',2,'MarkerSize',11);
        end
        ax.YScale='log'; ylim(ax,[.02 5]); xticks(ax,1:height(data)); xticklabels(ax,string(data.frame));
        grid(ax,'on'); xlabel(ax,'Query frame'); ylabel(ax,'Difference from recorded XY pose (m)');
        title(ax,replace(calibration,["identity",tag],["Identity frame transform","Explicit offline pitch candidate"]),'FontSize',12);
        legend(ax,handles,labels,'Location','northoutside','FontSize',8);
    end
    title(layout,{'Query excluded from six-frame map; crosses denote rejected full-pose measurements', ...
        'Initial offset [0.5 m, -0.4 m, 2 deg]; differences reference the recorded mapping trajectory'},'FontSize',12);
    svgPath=fullfile(exportFolder,'registration_comparison.svg');
    exportgraphics(figureHandle,svgPath,'ContentType','vector');
    svg=regexprep(fileread(svgPath),'[ \t]+(\r?\n)','$1');
    fid=fopen(svgPath,'w'); assert(fid>=0); closeFile=onCleanup(@() fclose(fid));
    fwrite(fid,svg); clear closeFile
    exportgraphics(figureHandle,fullfile(outputFolder,'registration_comparison.png'),'Resolution',150);
    clear closeFigure
    disp(summary.groups);
end
