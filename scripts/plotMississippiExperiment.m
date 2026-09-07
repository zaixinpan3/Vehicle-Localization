function fig=plotMississippiExperiment(folder)
% plotMississippiExperiment Display causal trajectories and retained failures.
    calls=readtable(fullfile(folder,'recursive_8threads','calls.csv'));
    fusion=readtable(fullfile(folder,'observer_fusion','online.csv'));
    gps=readtable(fullfile(folder,'observer_gpsOnly','online.csv'));
    outage=readtable(fullfile(folder,'observer_positionOutage','online.csv'));
    noLidar=readtable(fullfile(folder,'observer_outageNoLidar','online.csv'));
    failure=jsondecode(fileread(fullfile(folder,'observer_outageNoLidar','summary.json')));
    fusionFailure=jsondecode(fileread(fullfile(folder,'observer_positionOutage','summary.json')));
    fig=figure('Name','Mississippi full perception and observer experiment','Color','w', ...
        'Position',[40 40 1600 900]);
    layout=tiledlayout(fig,2,3,'TileSpacing','compact','Padding','compact');
    title(layout,'Mississippi 12:09:31: same-drive map, fresh perception, causal observer');
    origin=[calls.referenceX(1),calls.referenceY(1)];
    nexttile; plot(calls.referenceX-origin(1),calls.referenceY-origin(2),'k','LineWidth',1.5); hold on;
    plot(calls.x-origin(1),calls.y-origin(2),'Color',[.1 .5 .8]);
    plot(fusion.x-origin(1),fusion.y-origin(2),'Color',[.1 .65 .25]);
    axis equal; grid on; xlabel('Easting from start (m)'); ylabel('Northing from start (m)');
    legend('GNSS/INS','Recursive D2D','Causal observer','Location','best'); title('Trajectory');
    nexttile; plot(calls.timeSeconds,calls.positionErrorM,'Color',[.1 .5 .8]); hold on;
    plot(gps.time,gps.positionErrorM,'Color',[.8 .5 .1]);
    plot(fusion.time,fusion.positionErrorM,'Color',[.1 .65 .25]);
    grid on; xlabel('Receiver time (s)'); ylabel('Position error (m)');
    legend('D2D + prediction on rejection','GPS-only observer','Fusion observer','Location','best');
    title('All-time position consistency');
    nexttile; plot(calls.timeSeconds,calls.yawErrorDeg,'Color',[.1 .5 .8]); hold on;
    plot(fusion.time,fusion.yawErrorDeg,'Color',[.1 .65 .25]);
    grid on; xlabel('Receiver time (s)'); ylabel('Yaw error (deg)'); title('Heading consistency');
    nexttile; semilogy(outage.time,max(.001,outage.positionErrorM),'Color',[.1 .65 .25]); hold on;
    semilogy(noLidar.time,max(.001,noLidar.positionErrorM),'Color',[.85 .25 .1]);
    xline(40,'k--'); xline(60,'k--');
    if ~failure.completed
        xline(failure.firstFailingSampleTime,'r:',sprintf('Nonfinite at %.2f s',failure.firstFailingSampleTime));
    end
    if ~fusionFailure.completed
        xline(fusionFailure.firstFailingSampleTime,'g:',sprintf('Fusion fails at %.2f s',fusionFailure.firstFailingSampleTime));
    end
    xlim([35 65]); ylim([.01 300]); grid on;
    xlabel('Receiver time (s)'); ylabel('Position error (m), log scale');
    legend('With D2D','Without D2D','Location','best');
    title('GPS XY outage [40,60); errors above 300 m exceed view');
    nexttile; plot(calls.frame,calls.totalMs,'Color',[.2 .35 .7]); hold on; yline(100,'r--');
    grid on; xlabel('LiDAR frame'); ylabel('Computation (ms)');
    title('8 threads: map selection + perception + D2D + event');
    nexttile; hold on;
    for factor=[.7 1 1.3]
        path=fullfile(folder,sprintf('observer_fusion_%.1f',factor),'online.csv');
        if factor==1, path=fullfile(folder,'observer_fusion','online.csv'); end
        if ~isfile(path), continue; end
        trial=readtable(path);
        plot(trial.time,trial.positionErrorM,'DisplayName',sprintf('Dynamics scale %.1f',factor));
    end
    grid on; legend('Location','best'); xlabel('Receiver time (s)'); ylabel('Position error (m)');
    title('Unidentified inertia/stiffness: limited sensitivity');
    exportgraphics(fig,fullfile(folder,'overview.png'),'Resolution',170);
    exportgraphics(fig,fullfile(folder,'overview.pdf'),'ContentType','vector');
    exportgraphics(fig,fullfile(folder,'overview.svg'),'ContentType','vector');
end
