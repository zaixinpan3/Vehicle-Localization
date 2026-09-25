function plotPoleDistributionStatistics()
% plotPoleDistributionStatistics: Empirical CDFs of diagnostic reference groups.
    root=setupVehicleLocalization(); folder=fullfile(root,'research','pole_distribution_20260925');
    t=readtable(fullfile(folder,'reference_distributions.csv'));
    group={logical(t.fine), logical(t.baseline)&~logical(t.fine), ...
        logical(t.current)&~logical(t.baseline)&~logical(t.fine)};
    names={'Fine-confirmed (306)','Old coarse only (668)','Current only (272)'};
    variables={'coreFraction','maximumGap','gapRatio','interquartileHeight'};
    labels={'Fraction in fixed XY core','Largest height gap [m]', ...
        'Largest gap / height span','Interquartile height span [m]'};
    figure('Color','w','Position',[80 80 1100 750]); tiledlayout(2,2,'TileSpacing','compact');
    for j=1:4
        nexttile; hold on;
        for k=1:3
            values=sort(t.(variables{j})(group{k})); values=values(isfinite(values));
            stairs(values,(1:numel(values))/numel(values),'LineWidth',1.6);
        end
        xlabel(labels{j}); ylabel('Empirical cumulative fraction'); grid on; ylim([0 1]);
        if j==1, legend(names,'Location','northwest','FontSize',9); end
        if j==2, xlim([0 2]); elseif j==3, xlim([0 1]); end
    end
    sgtitle('117 Mississippi frames: reference agreement groups, not manual ground truth');
    exportgraphics(gcf,fullfile(folder,'distribution_statistics.png'),'Resolution',150);
end
