function plot_diagnosis()
% plot_diagnosis Display recorded errors and the specific suspect associations.
    dest=fileparts(mfilename('fullpath'));out=fullfile(pwd,'output/frame959_matching_diagnosis_20260922');
    s=load(fullfile(out,'diagnostic.mat'));p=load(fullfile(out,'pole_audit.mat'));
    call=readtable(fullfile(dest,'neighborhood.csv'));q=readtable(fullfile(dest,'iterations.csv'));
    fig=figure(Visible='off',Color='w',Position=[0 0 1300 900]);cleanup=onCleanup(@()close(fig));t=tiledlayout(fig,2,2);
    ax=nexttile(t);plot(ax,call.frame,100*call.positionErrorM,'-o',LineWidth=1.4);hold(ax,'on');
    predicted=hypot(call.predictedX-call.referenceX,call.predictedY-call.referenceY);plot(ax,call.frame,100*predicted,'--',LineWidth=1.2);
    xline(ax,959,':');grid(ax,'on');xlabel(ax,'Frame');ylabel(ax,'XY error (cm)');legend(ax,'After matching','Before matching');title(ax,'A bad match creates the peak');
    ax=nexttile(t);yyaxis(ax,'left');plot(ax,q.iteration,100*q.errorM,'-o',LineWidth=1.3);ylabel(ax,'Position error (cm)');
    yyaxis(ax,'right');plot(ax,q.iteration,q.similarity,'-s',LineWidth=1.3);ylabel(ax,'Reported similarity');grid(ax,'on');xlabel(ax,'Solver iteration');title(ax,'Cost falls while position and similarity worsen');
    ax=nexttile(t);R=[cos(s.reference(3)),-sin(s.reference(3));sin(s.reference(3)),cos(s.reference(3))];hold(ax,'on');
    selected=s.pairs.semanticName=="pole";source=[s.pairs.sourceX(selected),s.pairs.sourceY(selected)];target=s.pairs.targetBodyXY(selected,:);
    solved=(s.pairs.sourceAtSolution(selected,:)-s.reference(1:2))*R;
    plot(ax,source(:,1),source(:,2),'bo',DisplayName='Source at reference');plot(ax,target(:,1),target(:,2),'rx',MarkerSize=10,DisplayName='Chosen map target');
    plot(ax,solved(:,1),solved(:,2),'k+',DisplayName='Source at fitted pose');
    plot(ax,[source(:,1),target(:,1)].',[source(:,2),target(:,2)].',':',Color=[.5,.5,.5],HandleVisibility='off');
    ids=s.pairs.source(selected);far=ismember(ids,[100,142]);
    text(ax,source(far,1)+.3,source(far,2),string(ids(far)));
    text(ax,source(1,1)+.3,source(1,2),'36, 105, 141');axis(ax,'equal');grid(ax,'on');
    xlabel(ax,'Reference forward (m)');ylabel(ax,'Reference left (m)');legend(ax,Location='best');title(ax,'Pole correspondences; numbers are Gaussian IDs');
    ax=nexttile(t);n=p.rawNeighborhoods{2};delta=n.points-n.meanXYZ;
    scatter(ax,delta(:,1),n.points(:,3),25,[.25 .25 .25],'filled');grid(ax,'on');
    xlabel(ax,'Local X from component mean (m)');ylabel(ax,'Height from INS reference (m)');
    title(ax,{'Source 100: raw points within 0.7 m in XY','No points accepted as pole by fine perception'});
    exportgraphics(fig,fullfile(dest,'diagnosis.png'),Resolution=160);
end
