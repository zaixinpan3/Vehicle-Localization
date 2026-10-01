function verifySupportRepair()
% verifySupportRepair Verify complete causal replays and export the comparison.
    setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));
    old=readtable('research/source_shape_matching_20260929/final_raw.csv');
    legacy=readtable(fullfile(dest,'legacy_verified.csv'));
    current=readtable(fullfile(dest,'final_default.csv'));
    overlap=readtable(fullfile(dest,'overlap_verified.csv'));
    delta=max(abs(legacy{:,{'x','y','psi'}}-old{:,{'x','y','psi'}}),[],'all');
    assert(delta==0,'The retained geometricD2D comparator changed.');
    assert(height(current)==1170 && all(isfinite(current{:,{'x','y','psi','errorM'}}),'all'));
    assert(distributionRegistrationConfig().method=="supportD2D");
    assert(max(current.errorM(2:end))<=max(legacy.errorM(2:end)));
    assert(rms(current.errorM)<=rms(legacy.errorM));
    assert(max(current.errorM(2:end))<.3*max(overlap.errorM(2:end)));
    report=readtable(fullfile(dest,'legacy_verified_summary.csv'));
    report=[report;readtable(fullfile(dest,'overlap_verified_summary.csv'));readtable(fullfile(dest,'final_default_summary.csv'))];
    writetable(report,fullfile(dest,'comparison.csv'));
    frames=[178;894;895;932];
    focus=table(frames,legacy.errorM(frames),overlap.errorM(frames),current.errorM(frames), ...
        VariableNames={'frame','legacyErrorM','overlapErrorM','repairedErrorM'});
    writetable(focus,fullfile(dest,'focus_frames.csv'));
    fig=figure('Visible','off','Color','w','Position',[100 100 1300 550]);cleanup=onCleanup(@()close(fig));
    colororder(fig,[.10 .45 .80;.85 .30 .08;.10 .55 .25]);
    tiledlayout(1,2,Padding='compact',TileSpacing='compact');
    nexttile;plot(current.frame,100*[legacy.errorM,overlap.errorM,current.errorM]);
    xlim([2 1170]);ylim([0 65]);grid on;xlabel('Frame');ylabel('Horizontal discrepancy (cm)');
    legend('Previous geometric model','Full Gaussian overlap','Repaired support model',Location='northoutside');
    title('Complete causal Mississippi replay');
    nexttile;bar(categorical(string(frames)),100*[focus.legacyErrorM,focus.overlapErrorM,focus.repairedErrorM]);
    grid on;xlabel('Frame');ylabel('Horizontal discrepancy (cm)');title('Previously problematic frames');
    set(findall(fig,'Type','axes'),'Color','w','XColor','k','YColor','k','GridColor',[.65 .65 .65]);
    set(findall(fig,'Type','text'),'Color','k');set(findall(fig,'Type','legend'),'Color','w','TextColor','k');
    exportgraphics(fig,fullfile(dest,'route_comparison.png'),Resolution=150);
    exportgraphics(fig,fullfile(dest,'route_comparison.pdf'),ContentType='vector');
    validation=struct('frames',height(current),'referenceUsedForScoringOnly',true, ...
        'legacyMaximumPoseDifference',delta,'defaultMethod',"supportD2D", ...
        'maximumImprovedAgainstLegacy',true,'rmseImprovedAgainstLegacy',true, ...
        'rawPerceptionRerun',false,'runtimeControlled',false);
    fid=fopen(fullfile(dest,'replay_validation.json'),'w');fprintf(fid,'%s\n',jsonencode(validation,PrettyPrint=true));fclose(fid);
    disp(report);disp(focus);
end
