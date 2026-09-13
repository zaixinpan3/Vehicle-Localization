function fig=plotMississippiExperiment(folder)
% plotMississippiExperiment Display the two continuous reconstruction trials.
% Rejected measurement contracts remain visible as failed trials, without
% fabricating trajectories or substituting another measurement mode.
    arguments
        folder (1,1) string
    end
    fig=figure('Name','Continuous observer reconstruction experiment','Color','w', ...
        'Position',[40 40 1400 700]);
    layout=tiledlayout(fig,2,3,'TileSpacing','compact','Padding','compact');
    title(layout,'Offline continuous reconstruction: separate GNSS and fixed-delay LiDAR');
    for mode=["gnss","lidar"]
        trialFolder=fullfile(folder,"continuous_"+mode);
        summary=jsondecode(fileread(fullfile(trialFolder,'summary.json')));
        if ~summary.completed
            for panel=1:3
                ax=nexttile;axis(ax,'off');
                text(ax,.02,.65,mode+": reconstruction/run rejected",'Interpreter','none');
                text(ax,.02,.45,string(summary.errorIdentifier),'Interpreter','none');
            end
            continue;
        end
        trajectory=readtable(fullfile(trialFolder,'trajectory.csv'));
        origin=[trajectory.referenceX(1),trajectory.referenceY(1)];
        positionError=hypot(trajectory.x-trajectory.referenceX,trajectory.y-trajectory.referenceY);
        yawError=rad2deg(atan2(sin(trajectory.psi-trajectory.referencePsi),cos(trajectory.psi-trajectory.referencePsi)));
        nexttile;plot(trajectory.referenceX-origin(1),trajectory.referenceY-origin(2),'k');hold on;
        plot(trajectory.x-origin(1),trajectory.y-origin(2));axis equal;grid on;
        xlabel('Easting from start (m)');ylabel('Northing from start (m)');
        legend('GNSS/INS reference','Continuous observer','Location','best');title(mode+" trajectory");
        nexttile;plot(trajectory.time,positionError);grid on;
        xlabel('Receiver time (s)');ylabel('Position error (m)');title(mode+" position");
        nexttile;plot(trajectory.time,yawError);grid on;
        xlabel('Receiver time (s)');ylabel('Yaw error (deg)');title(mode+" heading");
    end
    exportgraphics(fig,fullfile(folder,'continuous_overview.png'),'Resolution',170);
    exportgraphics(fig,fullfile(folder,'continuous_overview.pdf'),'ContentType','vector');
end
