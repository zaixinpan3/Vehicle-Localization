function plot_results()
% plot_results Show sequence accuracy and actual temporal support processing.
    dest=fileparts(mfilename('fullpath'));c=readtable(fullfile(dest,'error_curves.csv'));
    w=readtable(fullfile(dest,'source_windows.csv'));figureHandle=figure(Visible='off',Position=[50 50 1250 850]);
    cleanup=onCleanup(@()close(figureHandle));tiledlayout(2,2,TileSpacing='compact');
    nexttile;plot(c.frame,100*c.concatenated_baselineErrorM,Color=[.65 .65 .65]);hold on;
    plot(c.frame,100*c.confirmed_three_framesErrorM,Color=[0 .35 .8]);
    xlabel('Frame');ylabel('Trajectory position error (cm)');title('All 1170 frames, including prediction-only startup');
    legend('Concatenated baseline','Confirmed three-frame horizon');grid on;
    nexttile;near=c.frame>=949 & c.frame<=965;
    plot(c.frame(near),100*c.concatenated_baselineErrorM(near),'-o',Color=[.65 .65 .65]);hold on;
    plot(c.frame(near),100*c.confirmed_three_framesErrorM(near),'-s',Color=[0 .35 .8]);
    xlabel('Frame');ylabel('Position error (cm)');title('Frames 958 and 959 now supply directional constraints');grid on;
    ylim([0 80]);legend('Concatenated baseline','Confirmed three-frame horizon',Location='northwest');
    nexttile;plot(w.frame,w.unfilteredComponents,Color=[.65 .65 .65]);hold on;
    plot(w.frame,w.components,Color=[0 .35 .8]);xlabel('Frame');ylabel('Gaussian distributions');
    legend('Input distributions across horizon','Merged distributions with repeated support');grid on;
    nexttile;plot(w.frame,w.windowMs);xlabel('Frame');ylabel('Temporal processing (ms)');
    title('Alignment, association, confirmation and moment merging');grid on;
    exportgraphics(figureHandle,fullfile(dest,'comparison.png'),Resolution=140);
end
