function plotLidarObserverDiagnosis()
% plotLidarObserverDiagnosis Export the measured error budget without a GUI.
    dest=fileparts(mfilename('fullpath'));
    d=readtable(fullfile(dest,'decomposition.csv'));
    m=readtable(fullfile(dest,'motion_inputs.csv'));
    original=readtable('research/support_full_localization_20260930/frame_errors.csv',TextType='string');
    original=original(original.scenario=="lidar_only",:);
    assert(isequal(d.frame,original.frame));
    fig=figure('Visible','off','Color','w','Position',[100 100 1350 850]);
    cleanup=onCleanup(@()close(fig));tiledlayout(2,2,Padding='compact');
    nexttile;ix=d.time>=2;plot(d.time(ix),100*original.positionErrorM(ix));hold on;
    plot(d.time(ix),100*original.matchedPositionErrorM(ix));grid on;
    ylabel('Position discrepancy (cm)');xlabel('Receiver time (s)');
    title('Same accepted LiDAR poses; observer includes motion');legend('Observer','Matching');
    nexttile;ix=d.time>=80 & d.time<=89;
    plot(d.time(ix),100*[d.errorX(ix),m.referenceKinematicsX(ix),d.quadratureX(ix)]);
    xline(d.time(847),'k:');grid on;ylabel('Easting contribution (cm)');xlabel('Receiver time (s)');
    title('Exact propagated contributions near frame 847');legend('Total error','Reference kinematics','Endpoint quadrature',Location='best');
    nexttile;terms=[m.referenceKinematicsX(847),m.referenceKinematicsY(847); ...
        d.quadratureX(847),d.quadratureY(847);d.filterX(847),d.filterY(847); ...
        m.longitudinalX(847)+m.lateralX(847)+m.headingX(847),m.longitudinalY(847)+m.lateralY(847)+m.headingY(847); ...
        d.matchX(847),d.matchY(847)];
    barh(100*terms);yticks(1:5);yticklabels({'INS position/velocity inconsistency','Endpoint quadrature','Velocity state filter','Body velocity + heading','Filtered matching'});
    grid on;xlabel('Signed position contribution (cm)');title('Frame 847: vectors add to [24.995, 8.203] cm');legend('Easting','Northing',Location='best');
    nexttile;ix=d.time>=80 & d.time<=89;
    plot(d.time(ix),m.lateralVy(ix));hold on;plot(d.time(ix),m.lateralVy(ix)+m.biasVy(ix));plot(d.time(ix),m.insBodyVy(ix));
    grid on;xlabel('Receiver time (s)');ylabel('Body lateral velocity (m/s)');
    title('Bias correction helps, but is not instantaneous');legend('Lateral observer','After LiDAR bias correction','INS diagnostic velocity',Location='best');
    set(findall(fig,'Type','axes'),'Color','w','XColor','k','YColor','k');
    set(findall(fig,'Type','text'),'Color','k');set(findall(fig,'Type','legend'),'Color','w','TextColor','k');
    exportgraphics(fig,fullfile(dest,'diagnosis.png'),Resolution=160);
    exportgraphics(fig,fullfile(dest,'diagnosis.pdf'),ContentType='vector');
end
