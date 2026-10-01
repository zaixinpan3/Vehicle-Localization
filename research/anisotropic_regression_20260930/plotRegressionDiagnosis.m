function plotRegressionDiagnosis()
% plotRegressionDiagnosis Export static causal-control and information charts.
    dest=fileparts(mfilename('fullpath'));c=readtable(fullfile(dest,'controls.csv'));
    v=readtable(fullfile(dest,'shape_curvature.csv'));
    modes=["full_fixed_new","position_fixed_new","frozen_cov_position_fixed_new", ...
        "full_fixed_old","full_direction_fixed_new","partial_direction_fixed_new"];
    values=zeros(numel(modes),2);
    for k=1:numel(modes)
        for j=1:2
            frame=[178 894];values(k,j)=100*c.errorM(c.frame==frame(j)&c.variant==modes(k));
        end
    end
    fig=figure('Visible','off','Color','w','Position',[50 50 1380 590]);cleanup=onCleanup(@()close(fig));
    tiledlayout(1,2,Padding='compact',TileSpacing='compact');
    nexttile;bar(values);grid on;ylabel('Horizontal discrepancy (cm)');title('Fixed-pair controls; same source weights');
    xticklabels({'Full overlap','No shape cost','Freeze covariance','Old pairs', ...
        'Restore neighborhood direction','Direction + partial sign'});xtickangle(30);legend('Frame 178','Frame 894',Location='northoutside');
    nexttile;curb=v(v.class=="curb",:);info=[curb.originalShapeGnInformation,curb.splitShapeGnInformation,curb.legacyNeighborhoodInformation];
    bar(info);set(gca,'YScale','log');grid on;xticklabels(string(curb.frame));xlabel('Frame');
    ylabel('Angular geometry information (rad^{-2}; uncalibrated)');title('Direction factors carry very different strength');
    legend('Original Gaussian shape GN','Split shape GN','Legacy neighborhood',Location='northoutside');
    set(findall(fig,'Type','axes'),'Color','w','XColor','k','YColor','k','GridColor',[.65 .65 .65]);
    set(findall(fig,'Type','text'),'Color','k');
    set(findall(fig,'Type','legend'),'Color','w','TextColor','k','EdgeColor',[.6 .6 .6]);
    exportgraphics(fig,fullfile(dest,'causal_controls.png'),Resolution=160);
    exportgraphics(fig,fullfile(dest,'causal_controls.pdf'),ContentType='vector');
end
