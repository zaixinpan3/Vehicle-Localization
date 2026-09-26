function plotPoleSubsetExamples()
% plotPoleSubsetExamples: Show minority shafts and the remaining label ambiguity.
    root=setupVehicleLocalization();folder=fullfile(root,'research','pole_subset_20260925');
    data=load(fullfile(root,'output','pole_distribution_20260925','point_distributions.mat'));
    tableData=readtable(fullfile(folder,'native_pillars.csv'));
    confirmed=tableData(tableData.accepted & tableData.fine,:);
    confirmed=sortrows(confirmed,'ownSupportFraction');
    novel=tableData(tableData.accepted & ~tableData.fine & ~tableData.previous,:);
    novel=sortrows(novel,'score','descend');
    selected=[confirmed(1,:);novel(1,:)];
    fig=figure('Visible','off','Position',[100 100 1300 1000]);layout=tiledlayout(fig,3,3,'TileSpacing','compact');
    z=linspace(-1,3,60).';angle=(1:60).'*2.399;rng(14);
    shaft=[.12+.025*cos(angle),.16+.025*sin(angle),z];
    clutter=[.04+.52*rand(800,2),3.6+.5*rand(800,1)];
    points=[shaft;clutter];g=struct('origin',[0 0],'cellSize',[.6 .6],'mapSize',[1 1]);
    diagnostics=drawExample(layout,points,ones(size(points,1),1),g,1,'Synthetic: 60 shaft + 800 clutter returns');
    diagnostics.frame=0;diagnostics.pillar=1;diagnostics.reference="synthetic";
    examples=diagnostics;
    for k=1:2
        item=selected(k,:);record=data.records{find(cellfun(@(r)r.frame,data.records)==item.frame,1)};
        label="Fine-reference overlap";if k==2,label="New candidate; no manual label";end
        titleText=sprintf('%s: frame %d, pillar %d',label,item.frame,item.pillarIndices);
        frame=loadPointCloudFrame(fullfile(root,'data','raw','MissisipiPointClouds.mat'),record.frame);
        index=double(record.originalIndices);points=double([frame.x(index),frame.y(index),frame.z(index)]);
        row=drawExample(layout,points,record.pillarIds,record.geometry,item.pillarIndices,titleText);
        row.frame=item.frame;row.pillar=item.pillarIndices;row.reference=label;examples(end+1)=row; %#ok<AGROW>
    end
    title(layout,'Evidence concerns a compact continuous subset of original returns');
    drawnow;
    exportgraphics(fig,fullfile(folder,'subset_examples.png'),'Resolution',150);close(fig);
    writetable(struct2table(examples),fullfile(folder,'example_measurements.csv'));
end

function summary=drawExample(layout,p,ids,g,target,titleText)
    ids=double(ids); occupied=unique(ids); evaluate=occupied==target; cfg=structuralPillarConfig(.6);
    c=cfg.pole.subset;c.useNativeKernels=perceptionNativeAvailable;
    e=findPillarPoleSubsets(p,ids,g,c,evaluate);j=find(evaluate);assert(e.found(j));
    [row,col]=ind2sub(g.mapSize,target);[r,cc]=ind2sub(g.mapSize,ids);reach=ceil(c.neighborhoodRadius./g.cellSize);
    near=abs(r-row)<=reach(2) & abs(cc-col)<=reach(1);q=p(near,:);own=ids(near)==target;
    residual=q(:,1:2)-e.axisXY(j,:)-(q(:,3)-e.axisZ(j)).*e.slopeXY(j,:);distance=vecnorm(residual,2,2);
    support=distance<=e.radius(j) & q(:,3)>=e.minimumZ(j) & q(:,3)<=e.maximumZ(j);
    for i=find(support).'
        height=abs(q(:,3)-q(i,3))<=c.heightHalfWindow;
        inner=nnz(height & distance<=e.radius(j));outer=nnz(height & distance<=2*e.radius(j));
        support(i)=inner>=c.minimumWindowPoints && 3*inner/max(outer-inner,1)>=c.minimumContrast;
    end
    assert(nnz(support & own)==e.ownCount(j));
    color=[.78 .78 .78];highlight=[.8 .27 .05];local=q(:,1:2)-g.origin-([col row]-1).*g.cellSize;
    ax=nexttile(layout);scatter(ax,local(own,1),local(own,2),10,color,'filled');hold(ax,'on');
    scatter(ax,local(support & own,1),local(support & own,2),18,highlight,'filled');
    axis(ax,'equal');grid(ax,'on');xlim(ax,[0 g.cellSize(1)]);ylim(ax,[0 g.cellSize(2)]);
    xlabel(ax,'Within-pillar X [m]');ylabel(ax,'Within-pillar Y [m]');
    title(ax,replace(string(titleText),": frame",newline+"Frame"),'Interpreter','none','FontSize',10);
    ax=nexttile(layout);scatter(ax,distance(own),q(own,3),10,color,'filled');hold(ax,'on');
    scatter(ax,distance(support & own),q(support & own,3),16,highlight,'filled');
    scatter(ax,distance(support & ~own),q(support & ~own,3),16,[.1 .4 .7],'filled');
    xline(ax,e.radius(j),'--');grid(ax,'on');
    if contains(titleText,'Synthetic')
        text(ax,.35,.27,{'Gray: all owner returns';'Orange: owner support';'Blue: neighbor support'}, ...
            'Units','normalized','FontSize',9,'BackgroundColor','white');
    end
    xlabel(ax,'Distance to fitted axis [m]');ylabel(ax,'Height [m]');
    title(ax,sprintf('%d/%d own points support shaft (%.1f%%)',nnz(support & own),nnz(own),100*nnz(support & own)/nnz(own)));
    ax=nexttile(layout);heights=sort(q(support,3));plot(ax,heights(2:end),diff(heights),'.-','Color',highlight);grid(ax,'on');
    yline(ax,c.maximumGap+c.rangeGapScale*norm(e.axisXY(j,:)),'--');
    xlabel(ax,'Support height [m]');ylabel(ax,'Successive height gap [m]');
    title(ax,sprintf('Span %.2f m; RMS radius %.3f m; score %.2f',e.height(j),e.radialRms(j),e.score(j)));
    summary=struct('ownPoints',nnz(own),'ownSupport',nnz(support & own),'supportPoints',nnz(support), ...
        'ownFraction',nnz(support & own)/nnz(own),'height',e.height(j),'robustHeight',e.robustHeight(j), ...
        'radialRms',e.radialRms(j),'maximumGap',e.maximumGap(j),'score',e.score(j));
end
