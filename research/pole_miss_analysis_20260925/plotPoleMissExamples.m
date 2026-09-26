function plotPoleMissExamples()
% plotPoleMissExamples: Inspect original returns at five distinct rejection mechanisms.
    root=setupVehicleLocalization();folder=fileparts(mfilename('fullpath'));addpath(folder);
    data=load(fullfile(root,'output','pole_miss_analysis_20260925','cases.mat'));
    measurements=readtable(fullfile(folder,'reference_cell_causes.csv'));
    keys=[1 5789;301 4944;501 7257;851 4166;1031 4718];
    labels=["Default: global isolation veto","Subset: height below 1.5 m", ...
        "Subset: radial RMS just above 0.10 m","Subset: evidence found, footprint removed", ...
        "Subset: local density-peak veto"];
    fig=figure('Visible','off','Position',[50 50 1400 1900]);layout=tiledlayout(fig,5,2,'TileSpacing','compact');
    for k=1:size(keys,1)
        r=data.cases{find(cellfun(@(x)x.frame,data.cases)==keys(k,1),1)};
        id=keys(k,2);g=r.geometry;[row,col]=ind2sub(g.mapSize,id);
        ownFine=r.fineIds==id;center=median(r.finePoints(ownFine,1:2),1);
        fineNear=vecnorm(r.finePoints(:,1:2)-center,2,2)<=.5;q=r.finePoints(fineNear,:);
        z0=median(q(:,3));beta=[ones(size(q,1),1),q(:,3)-z0]\q(:,1:2);
        near=all(abs(r.points(:,1:2)-center)<.75,2);points=r.points(near,:);
        stats=measurements(measurements.frame==r.frame & measurements.pillarIndices==id,:);
        e=r.after.columnMaps.poleSubset;accepted=ismember(e.pillarIndices,find(r.after.poleCellMask));
        projected=e.axisXY+(z0-e.axisZ).*e.slopeXY;
        localAccepted=accepted & all(abs(projected-center)<.75,2);
        ax=nexttile(layout);scatter(ax,points(:,1)-center(1),points(:,2)-center(2),7,[.75 .75 .75],'filled');hold(ax,'on');
        scatter(ax,q(:,1)-center(1),q(:,2)-center(2),13,[.85 .3 .05],'filled');
        lower=g.origin+([col row]-1).*g.cellSize-center;
        rectangle(ax,'Position',[lower,g.cellSize],'EdgeColor',[.7 .1 .1],'LineWidth',1.3);
        [rr,cc]=ind2sub(g.mapSize,e.pillarIndices(localAccepted));
        for j=1:numel(rr)
            low=g.origin+([cc(j) rr(j)]-1).*g.cellSize-center;
            rectangle(ax,'Position',[low,g.cellSize],'EdgeColor',[.1 .35 .8],'LineStyle','--');
        end
        scatter(ax,projected(localAccepted,1)-center(1),projected(localAccepted,2)-center(2),35,[.1 .35 .8],'x','LineWidth',1.5);
        axis(ax,'equal');xlim(ax,[-.75 .75]);ylim(ax,[-.75 .75]);grid(ax,'on');
        xlabel(ax,'Relative X [m]');ylabel(ax,'Relative Y [m]');
        title(ax,sprintf('%s\nFrame %d / pillar %d',labels(k),r.frame,id),'FontSize',10);
        ax=nexttile(layout);distance=vecnorm(points(:,1:2)-beta(1,:)-(points(:,3)-z0).*beta(2,:),2,2);
        fineDistance=vecnorm(q(:,1:2)-beta(1,:)-(q(:,3)-z0).*beta(2,:),2,2);
        scatter(ax,distance,points(:,3),7,[.75 .75 .75],'filled');hold(ax,'on');
        scatter(ax,fineDistance,q(:,3),13,[.85 .3 .05],'filled');
        for j=find(localAccepted).'
            zz=linspace(e.minimumZ(j),e.maximumZ(j),40).';xy=e.axisXY(j,:)+(zz-e.axisZ(j)).*e.slopeXY(j,:);
            deviation=vecnorm(xy-beta(1,:)-(zz-z0).*beta(2,:),2,2);
            plot(ax,deviation,zz,'Color',[.1 .35 .8],'LineWidth',1.3);
        end
        grid(ax,'on');xlabel(ax,'Distance from diagnostic fine-reference axis [m]');ylabel(ax,'Height [m]');
        isolation=compose("%.3f",stats.coreIsolation);
        if stats.failCount || stats.failHeight,isolation="not evaluated";end
        title(ax,sprintf('Fine points %d; isolation %s; subset stage %d',stats.fineOriginalOwnCount,isolation,stats.stage),'FontSize',10);
    end
    title(layout,{'Pole-loss mechanisms on original recorded returns', ...
        'Gray: off-ground returns; orange: fine algorithm labels; blue: accepted subset shafts/cells; red: target cell'},'FontSize',11);
    drawnow;exportgraphics(fig,fullfile(folder,'miss_mechanisms.png'),'Resolution',140);close(fig);
end
