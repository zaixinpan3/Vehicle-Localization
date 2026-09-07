function fig=plotMncavOutageDiagnosis(folder)
% plotMncavOutageDiagnosis Compare saved controlled observer counterfactuals.
    arguments
        folder (1,1) string
    end
    traces=readtable(fullfile(folder,'error_traces.csv'),'TextType','string');
    summary=readtable(fullfile(folder,'ablations.csv'),'TextType','string');
    fig=figure('Color','w','Position',[80 80 1400 800]);
    tiledlayout(fig,2,2,'TileSpacing','compact');
    groups={["baseline","perfectPose","zeroDelay","substeps4"], ...
        ["baseline","unitWeight","hold120ms","basePositionGain"], ...
        ["baseline","baseGainUnitWeight","noInvariant"]};
    titles=["Measurement and integration ablations","Gain and hold ablations", ...
        "Finite arithmetic does not imply convergence"];
    for k=1:3
        nexttile;hold on;
        for name=groups{k}
            part=traces(traces.caseName==name,:);
            plot(part.time,max(part.positionErrorM,1e-4),'DisplayName',name);
        end
        set(gca,'YScale','log','YTick',10.^(-2:4));grid on;xlim([39 61]);ylim([.01 1e4]);
        xline(40,'k:','GNSS XY off','HandleVisibility','off','LabelOrientation','horizontal');
        xline(60,'k:','on','HandleVisibility','off','LabelOrientation','horizontal');
        xlabel('Time (s)');ylabel('Position discrepancy (m)');title(titles(k));
        legend('Location','northwest','Interpreter','none');
    end
    nexttile;
    failed=summary(~summary.completedTo61s,:);
    barh(categorical(failed.caseName),failed.firstNonfiniteTime-40);
    xlabel('Seconds from GNSS XY removal to nonfinite state');
    title('Failures retained; finite traces stop before overflow');
    grid on;
    exportgraphics(fig,fullfile(folder,'diagnosis.png'),'Resolution',170);
    exportgraphics(fig,fullfile(folder,'diagnosis.pdf'),'ContentType','vector');
end
