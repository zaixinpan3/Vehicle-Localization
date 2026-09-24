function plotAmbiguityStudy()
% plotAmbiguityStudy Error traces of the current, canonical and pyramid matchers.
    dest=fileparts(mfilename('fullpath'));
    A=load('output/alias_hypotheses_20260924/canonicalized.mat','results');P=load('output/alias_hypotheses_20260924/pyramid.mat','results');
    current=A.results.map000_source000.loopM;canonical=A.results.map150_source050.loopM;fine=P.results.config5.loop(:,2);
    t=(1:numel(current))/10;
    fig=figure('Visible','off','Color','w','Position',[100 100 1250 520]);theme(fig,'light');hold on;
    plot(t,100*current,Color=[.55 .55 .55],DisplayName='current single-seed solver');
    plot(t,100*canonical,Color=[.1 .45 .85],DisplayName='canonical clouds (map 1.5 m, source 0.5 m)');
    plot(t,100*fine,Color=[.85 .25 .1],DisplayName='pyramid: canonical then original (ungated)');
    ylabel('LiDAR-only matching position error (cm)');xlabel('Time since first scan (s)');ylim([0 75]);grid on;legend(Location='northeast');
    title('Closed-loop recursive LiDAR-only matching, 1170 scans');
    exportgraphics(fig,fullfile(dest,'ambiguity_study_traces.png'),Resolution=140);close(fig);
end
