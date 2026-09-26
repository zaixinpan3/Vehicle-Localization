function plotShaftStudy()
% plotShaftStudy: Render support statistics, without treating reference labels as truth.
    root=setupVehicleLocalization();folder=fileparts(mfilename('fullpath'));
    data=load(fullfile(root,'output','pole_miss_analysis_20260925','cases.mat'));
    z=linspace(-1,3,60).';angle=(1:60).'*2.399;rng(14);
    shaft=[.12+.025*cos(angle),.16+.025*sin(angle),z];
    clutter=[.04+.52*rand(800,2),3.6+.5*rand(800,1)];
    synthetic=struct('points',[shaft;clutter],'pillarIds',ones(860,1), ...
        'geometry',struct('origin',[0 0],'cellSize',[.6 .6],'mapSize',[1 1]));
    cases={synthetic,data.cases{1},data.cases{31}};targets=[1 5789 4944];
    labels=["Synthetic: minority shaft with dense end clutter","Recorded frame 1 / pillar 5789","Recorded frame 301 / pillar 4944"];
    cfg=structuralPillarConfig(.6);cfg.pole.detector="shaft";cfg.pole.probabilityEvidence="shaft";cfg.useNativeKernels=true;
    cloud=coarseSemanticProbabilityCloudConfig();cloud.semanticNames="pole";
    fig=figure('Visible','off','Position',[50 50 1200 1300]);layout=tiledlayout(fig,3,2,'TileSpacing','compact');
    measured=cell(3,1);
    for k=1:3
        r=cases{k};grid=struct('points',r.points,'pointPillarLinIdx',r.pillarIds,'pointAttributes',struct(),'pillarGeometry',r.geometry);
        result=analyzeStructuralPillars(grid,cfg,cloud);e=result.columnMaps.poleSubset;j=find(e.pillarIndices==targets(k));
        assert(e.found(j));p=r.points(r.pillarIds==targets(k),:);center=e.axisXY(j,:);
        d=vecnorm(p(:,1:2)-center-(p(:,3)-e.axisZ(j)).*e.slopeXY(j,:),2,2);
        support=d<=e.radius(j) & p(:,3)>=e.minimumZ(j) & p(:,3)<=e.maximumZ(j);
        ax=nexttile(layout);scatter(ax,p(:,1)-center(1),p(:,2)-center(2),9,[.7 .7 .7],'filled');hold(ax,'on');
        scatter(ax,p(support,1)-center(1),p(support,2)-center(2),18,[.88 .32 .04],'filled');
        zz=linspace(e.minimumZ(j),e.maximumZ(j),50).';xy=(zz-e.axisZ(j)).*e.slopeXY(j,:);
        plot(ax,xy(:,1),xy(:,2),'Color',[.1 .35 .8],'LineWidth',1.5);
        axis(ax,'equal');gridOn(ax);xlabel(ax,'X relative to shaft axis [m]');ylabel(ax,'Y relative to shaft axis [m]');title(ax,labels(k));
        ax=nexttile(layout);scatter(ax,d,p(:,3),9,[.7 .7 .7],'filled');hold(ax,'on');
        scatter(ax,d(support),p(support,3),18,[.88 .32 .04],'filled');
        plot(ax,[e.radius(j) e.radius(j)],[e.minimumZ(j) e.maximumZ(j)],'--','Color',[.1 .35 .8],'LineWidth',1.4);
        gridOn(ax);xlabel(ax,'Distance from shaft axis [m]');ylabel(ax,'Height [m]');
        title(ax,sprintf('Owner support %d/%d; span %.2f m; score %.3f',nnz(support),size(p,1),e.height(j),e.score(j)));
        measured{k}=struct('case',labels(k),'ownPoints',size(p,1),'supportedOwnPoints',nnz(support), ...
            'span',e.height(j),'score',e.score(j),'radialRms',e.radialRms(j),'radius',e.radius(j), ...
            'radialExcessScore',e.radialSignificance(j));
    end
    title(layout,{'Bounded pole-shaft evidence within existing 0.6 m pillars', ...
        'Gray: all owner returns; orange: selected support; blue: shaft axis or support radius'});
    drawnow;exportgraphics(fig,fullfile(folder,'shaft_examples.png'),'Resolution',140);close(fig);
    writetable(struct2table(vertcat(measured{:})),fullfile(folder,'example_measurements.csv'));
end
function gridOn(ax)
    grid(ax,'on');
end
