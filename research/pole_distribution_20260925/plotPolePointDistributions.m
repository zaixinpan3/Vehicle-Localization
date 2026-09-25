function plotPolePointDistributions()
% plotPolePointDistributions: Inspect owning pillar and context in metric space.
    root=setupVehicleLocalization();
    s=load(fullfile(root,'output','pole_distribution_20260925','point_distributions.mat'),'records');
    examples=[1 3846;1 5789;121 2340;891 6344;741 5657;961 2233];
    fig=figure('Visible','off','Position',[20 20 1600 1600],'Color','white');
    cleanup=onCleanup(@()close(fig));
    tiledlayout(size(examples,1),3,'TileSpacing','compact','Padding','compact');
    for k=1:size(examples,1)
        r=s.records{find(cellfun(@(x)x.frame,s.records)==examples(k,1),1)};
        id=examples(k,2); own=r.pillarIds==id; [row,col]=ind2sub(r.geometry.mapSize,id);
        center=r.geometry.origin+([col row]-0.5).*r.geometry.cellSize;
        xyz=double(r.points); xyz(:,1:2)=xyz(:,1:2)-center;
        near=all(abs(xyz(:,1:2))<=0.9,2); good=r.finePointMask & near;
        nexttile; scatter3(xyz(near,1),xyz(near,2),xyz(near,3),4,[.75 .75 .75],'filled');hold on;
        scatter3(xyz(own,1),xyz(own,2),xyz(own,3),12,[.8 .2 .1],'filled');
        scatter3(xyz(good,1),xyz(good,2),xyz(good,3),18,[0 .5 .1],'filled');
        view(35,20); grid on; xlabel('local X [m]');ylabel('local Y [m]');zlabel('Z [m]');
        title(sprintf('Frame %d, pillar %d: base %d, fine %d, detected %d',r.frame,id, ...
            ismember(id,r.baselineCells),ismember(id,r.fineCells),ismember(id,r.currentCells)));
        nexttile; scatter(xyz(near,1),xyz(near,2),5,[.75 .75 .75],'filled');hold on;
        scatter(xyz(own,1),xyz(own,2),15,xyz(own,3),'filled');
        rectangle('Position',[-.3 -.3 .6 .6],'EdgeColor','black');
        axis equal; xlim([-.9 .9]);ylim([-.9 .9]);grid on;colorbar;
        xlabel('local X [m]');ylabel('local Y [m]');title('Whole pillar: color = height');
        nexttile; scatter(xyz(near,1),xyz(near,3),5,[.75 .75 .75],'filled');hold on;
        scatter(xyz(own,1),xyz(own,3),14,[.8 .2 .1],'filled');
        scatter(xyz(good,1),xyz(good,3),18,[0 .5 .1],'filled');
        grid on; xlabel('local X [m]');ylabel('Z [m]');title('Height profile; green = fine reference');
    end
    exportgraphics(fig,fullfile(root,'research','pole_distribution_20260925','point_examples.png'),'Resolution',150);
end
