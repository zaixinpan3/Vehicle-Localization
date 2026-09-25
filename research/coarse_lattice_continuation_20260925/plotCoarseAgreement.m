function plotCoarseAgreement()
% plotCoarseAgreement: Show both parts of the per-channel acceptance decision.
    root=setupVehicleLocalization();
    folder=fullfile(root,'research','coarse_lattice_continuation_20260925');
    before=readtable(fullfile(folder,'takeover_agreement_by_split.csv'),'TextType','string');
    after=readtable(fullfile(folder,'agreement_by_split.csv'),'TextType','string');
    before=before(before.split=="all",:); after=after(after.split=="all",:);
    fig=figure('Visible','off','Position',[100 100 1200 470],'Color','white');
    cleanup=onCleanup(@()close(fig));
    tiledlayout(fig,1,2,'TileSpacing','compact','Padding','compact');
    metrics=["precisionTol","recallTol"]; titles=["Precision: suppress added cells","Recall: preserve baseline cells"];
    for k=1:2
        ax=nexttile; bar(ax,[before.(metrics(k)) after.(metrics(k))]);
        yline(ax,0.8,'k--','80% target','LineWidth',1.3);
        xticks(ax,1:4); xticklabels(ax,{'Curb','Pole','Traffic sign','Road'});
        ylim(ax,[0 1.08]); ylabel(ax,'Agreement fraction'); grid(ax,'on'); title(ax,titles(k));
        legend(ax,{'Takeover version','Metric-core update'},'Location','southoutside');
    end
    sgtitle('0.6 m vs frozen 0.3 m: all 1,170 scans, one-cell tolerance');
    exportgraphics(fig,fullfile(folder,'agreement.png'),'Resolution',150);
end
