function plot_review()
% plot_review Export trajectory context and the remaining sign ambiguity.
    dest=fileparts(mfilename('fullpath'));
    s=load('output/frame959_five_frame_review_20260922/review.mat');
    old=load('output/lidar_origin_20260922/coarse_pipeline/matching/report.mat','report');
    nearby=readtable(fullfile(dest,'neighborhood.csv'));
    targets=readtable(fullfile(dest,'sign_targets.csv'));
    fig=figure(Visible='off',Color='w',Position=[0 0 1350 900]);cleanup=onCleanup(@()close(fig));
    layout=tiledlayout(fig,2,2,TileSpacing='loose',Padding='loose');
    ax=nexttile(layout,[1 2]);hold(ax,'on');
    plot(ax,nearby.frame,100*old.report.calls.positionErrorM(nearby.frame),'-',LineWidth=1.6);
    plot(ax,nearby.frame,100*nearby.positionErrorM,'-o',LineWidth=1.6);
    plot(ax,nearby.frame,100*nearby.initialErrorM,'--',LineWidth=1.2);
    xline(ax,959,':');grid(ax,'on');xlabel(ax,'Frame');ylabel(ax,'Position error (cm)');
    title(ax,'The former frame-959 spike');legend(ax,'Before temporal confirmation','Five-frame matching','Five-frame prediction',Location='northwest');
    ax=nexttile(layout);values=100*[s.summary.oldErrorM;s.controls.errorM([1 2 3 4 12])];
    barh(ax,values);yticks(ax,1:6);yticklabels(ax,{'Old result','Current result','Use old seed', ...
        'Exclude poles','Exclude signs','Exclude sign target'});
    set(ax,YDir='reverse');xlabel(ax,'Frame-959 position error (cm)');grid(ax,'on');
    xlim(ax,[0 80]);text(ax,values+1,(1:6).',compose('%.2f',values));title(ax,{'Diagnostic interventions','All listed candidates accepted as full poses'});
    ax=nexttile(layout);hold(ax,'on');
    xy=s.source.components.mean(s.source.components.semanticName=="trafficSign",:);
    scatter(ax,xy(1),xy(2),75,[.1 .3 .8],'filled');
    scatter(ax,targets.bodyX(1),targets.bodyY(1),90,[.8 .15 .1],'x',LineWidth=2);
    scatter(ax,targets.bodyX(2),targets.bodyY(2),90,[.1 .6 .25],'+',LineWidth=2);
    for k=1:2
        plot(ax,[xy(1),targets.bodyX(k)],[xy(2),targets.bodyY(k)],':k',HandleVisibility='off');
    end
    axis(ax,'equal');grid(ax,'on');xlabel(ax,'Reference forward (m)');ylabel(ax,'Reference left (m)');
    title(ax,{'One sign detected in all five scans','Target distance: 52.9 cm versus 7.4 cm'});
    legend(ax,'Source at reference','Target from current seed','Target from old seed',Location='southwest');
    drawnow;
    exportgraphics(fig,fullfile(dest,'review.png'),Resolution=160);
end
