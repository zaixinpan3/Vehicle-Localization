function plot_results()
% plot_results Export complete matching and fusion error comparisons.
    dest=fileparts(mfilename('fullpath'));
    folders=["output/receiver_clock_20260921/coarse_pipeline", ...
        "output/lidar_origin_20260922/coarse_pipeline", ...
        "output/lidar_origin_20260922/independent_pipeline"];
    names=["Identity baseline","Selected sequence fit","Independent-drive fit"];
    fig=figure(Visible='off',Color='w',Position=[0 0 1100 700]);cleanup=onCleanup(@()close(fig));
    layout=tiledlayout(fig,2,1,TileSpacing='compact');rawAxes=nexttile(layout);hold(rawAxes,'on');
    for k=1:3
        calls=readtable(fullfile(folders(k),'matching/calls.csv'));
        plot(rawAxes,calls.frame,100*calls.positionErrorM,DisplayName=names(k));
    end
    ylabel(rawAxes,'Raw XY error (cm)');xlabel(rawAxes,'Frame');grid(rawAxes,'on');legend(rawAxes,Location='northeast');
    title(rawAxes,'1170 matching attempts; reference-origin correction applied to map and query');
    fusedAxes=nexttile(layout);hold(fusedAxes,'on');curves=[];
    for k=1:3
        result=load(fullfile(folders(k),'observer/experiment.mat'),'runs','reference','data');
        values=vecnorm(result.runs{1}.estimate.position-result.reference(:,1:2),2,2);
        plot(fusedAxes,result.data.highRate.time,100*values,DisplayName=names(k));
        if k==1,curves=result.data.highRate.time;end
        curves(:,k+1)=values; %#ok<AGROW>
    end
    ylabel(fusedAxes,'GNSS + LiDAR XY error (cm)');xlabel(fusedAxes,'Time (s)');grid(fusedAxes,'on');
    title(fusedAxes,'1169 fused samples; same-drive reference consistency, not independent accuracy');
    exportgraphics(fig,fullfile(dest,'comparison.png'),Resolution=160);
    writetable(array2table(curves,VariableNames={'timeSeconds','identityBaselineM','selectedSequenceFitM','independentDriveFitM'}), ...
        fullfile(dest,'fused_error_curve.csv'));
end
