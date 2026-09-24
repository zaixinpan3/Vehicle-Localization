function plotGnssLoss()
% plotGnssLoss Plot representative GNSS-loss error traces from the saved replay.
    dest=fileparts(mfilename('fullpath'));
    load('output/gnss_loss_20260923/traces.mat','traces','masks','patterns','seeds','t');
    t=t-t(1);
    fig=figure('Visible','off','Color','w','Position',[100,100,1250,900]);theme(fig,'light');tiledlayout(4,1,TileSpacing='compact');
    block=patterns(startsWith(patterns,"block_"));
    show={"iid_frames",1;"burst_1s",1;"burst_5s",1;block(1),45};
    none=traces(:,patterns=="none");
    for j=1:size(show,1)
        nexttile;r=find(patterns==show{j,1} & seeds==show{j,2},1);hold on;
        y=[0 80];lost=masks(:,r);d=diff([0;lost;0]);a=find(d==1);b=find(d==-1)-1;
        for i=1:numel(a)
            patch(t([a(i) b(i) b(i) a(i)]),[y(1) y(1) y(2) y(2)],[.9 .9 .9],EdgeColor='none',HandleVisibility='off');
        end
        plot(t,100*none,Color=[.6 .6 .6],DisplayName='no GNSS loss');
        plot(t,100*traces(:,r),Color=[.1 .3 .7],DisplayName=string(show{j,1}));
        ylim([0 80]);ylabel('Position error (cm)');grid on;box on;
        legend(Interpreter='none',Location='northeast');
        title(sprintf('%s (seed %g); shaded frames have no GNSS',show{j,1},show{j,2}),Interpreter='none');
    end
    xlabel('Time since first scan (s)');
    exportgraphics(fig,fullfile(dest,'gnss_loss_traces.png'),Resolution=150);close(fig);
end
