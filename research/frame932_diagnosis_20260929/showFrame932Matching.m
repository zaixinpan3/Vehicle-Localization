function fig=showFrame932Matching()
% showFrame932Matching Display fixed-map geometry and history transport bias.
% This diagram complements the original full-cloud feature/pillar viewer.
    setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));
    s=load('output/frame932_diagnosis_20260929/diagnostic.mat');
    t=load('output/frame932_diagnosis_20260929/trace.mat');
    fig=figure('Name','Mississippi 932 | matching error diagnosis','Theme','light','Color','w','Position',[80 80 1500 780]);
    layout=tiledlayout(fig,1,2,'TileSpacing','compact','Padding','compact');
    ax=nexttile(layout);hold(ax,'on');grid(ax,'on');axis(ax,'equal');
    plot(ax,t.fineCurb(:,1),t.fineCurb(:,2),'.','Color',[0 .6 .7],'MarkerSize',5,'DisplayName','Current original fine curb points');
    plot(ax,t.finePole(:,1),t.finePole(:,2),'.','Color',[1 .43 .1],'MarkerSize',5,'DisplayName','Current original fine pole points');
    pair=s.results{1}.correspondences;targets=unique(pair.globalTarget(pair.semanticName=="curb"));
    for k=1:numel(targets)
        id=targets(k);[v,e]=eig(s.fixed.components.covariance(:,:,id),'vector');[major,j]=max(e);
        axisXY=v(:,j).'*rot(s.ref(3));center=(s.fixed.components.mean(id,:)-s.ref(1:2))*rot(s.ref(3));
        endpoints=center+[-2;2]*sqrt(major)*axisXY;
        line(ax,endpoints(:,1),endpoints(:,2),'Color',[.05 .25 .45],'LineWidth',2,'HandleVisibility','off');
        text(ax,center(1),center(2)+.25,"Map curb "+id,'FontSize',10);
    end
    curb=s.source.components.semanticName=="curb";pole=s.source.components.semanticName=="pole";
    meanXY=s.source.components.mean;error=s.results{1}.poseXYTheta-s.ref;
    registered=meanXY*rot(error(3)).'+error(1:2)*rot(s.ref(3));
    plot(ax,meanXY(curb,1),meanXY(curb,2),'o','Color',[.85 .25 .15],'MarkerSize',5,'DisplayName','Confirmed coarse curb at reference pose');
    plot(ax,registered(curb,1),registered(curb,2),'x','Color',[.55 .1 .75],'MarkerSize',6,'DisplayName','Coarse curb at matched pose');
    plot(ax,meanXY(pole,1),meanXY(pole,2),'o','Color',[1 .43 .1],'MarkerSize',6,'DisplayName','History-confirmed pole (absent in current selection)');
    target=pair.targetMeanXY(pair.semanticName=="pole",:);target=(target-s.ref(1:2))*rot(s.ref(3));
    plot(ax,target(:,1),target(:,2),'+','Color',[.1 .1 .1],'MarkerSize',8,'LineWidth',1.5,'DisplayName','Map pole');
    xlim(ax,[0 19]);ylim(ax,[-4 8]);xlabel(ax,'Reference body X (m)');ylabel(ax,'Reference body Y (m)');
    title(ax,{'Frame 932: 15 confirmed curbs + 1 historical pole',sprintf('Origin error %.1f cm; yaw error %.3f deg',100*norm(error(1:2)),rad2deg(error(3)))});
    legend(ax,'Location','southoutside','FontSize',9);
    ax=nexttile(layout);hold(ax,'on');grid(ax,'on');
    motion=t.motion;plot(ax,motion.ageSeconds,100*motion.translationDeltaY,'o-','LineWidth',1.5,'DisplayName','Reference minus odometry: lateral');
    plot(ax,motion.ageSeconds,100*motion.translationDeltaX,'s-','LineWidth',1.5,'DisplayName','Reference minus odometry: longitudinal');
    xlabel(ax,'Acquisition age at frame 932 (s)');ylabel(ax,'Relative translation discrepancy (cm)');
    title(ax,{'History transport differs systematically from reference', ...
        'Reference-translation-only control: 16.0 cm -> 6.3 cm'});
    legend(ax,'Location','southoutside');
    annotation(fig,'textbox',[.64 .57 .30 .20],'String', ...
        sprintf(['Offline diagnostic controls:\n' ...
        'Production: 15.99 cm\nReference motion: 5.76 cm\n' ...
        'Remove curb 711: 10.61 cm\n' ...
        'Reference seed: returns to 15.99 cm']), ...
        'FitBoxToText','off','BackgroundColor','w','EdgeColor',[.7 .7 .7],'FontSize',10);
    drawnow;exportgraphics(fig,fullfile(dest,'matching_geometry.png'),'Resolution',160);
    exportgraphics(fig,fullfile(dest,'matching_geometry.pdf'),'ContentType','vector');
    fprintf('FRAME932_MATCHING_GUI_READY\n');
end
function r=rot(a)
    r=[cos(a) -sin(a);sin(a) cos(a)];
end
