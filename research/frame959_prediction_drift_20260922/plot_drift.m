function plot_drift()
% plot_drift Show signed accounting and recursive diagnostic controls.
    dest=fileparts(mfilename('fullpath'));
    steps=readtable(fullfile(dest,'steps.csv'));budget=readtable(fullfile(dest,'error_budget.csv'));
    controls=readtable(fullfile(dest,'controls.csv'),TextType='string');
    inputs=readtable(fullfile(dest,'motion_inputs.csv'));signs=readtable(fullfile(dest,'sign_associations.csv'),TextType='string');
    fig=figure(Visible='off',Color='w',Position=[0 0 1300 920]);cleanup=onCleanup(@()close(fig));
    layout=tiledlayout(fig,2,2,TileSpacing='loose',Padding='loose');
    ax=nexttile(layout);hold(ax,'on');
    plot(ax,steps.frame,100*steps.predictedErrorM,'--o',LineWidth=1.3);
    plot(ax,steps.frame,100*steps.matchedErrorM,'-o',LineWidth=1.3);
    grid(ax,'on');xlabel(ax,'Frame');ylabel(ax,'Position error (cm)');title(ax,'Prediction starts at the previous matched pose');
    legend(ax,'Incoming prediction','After matching',Location='northwest');
    ax=nexttile(layout);bar(ax,100*budget{:,2:3});
    xticks(ax,1:height(budget));xticklabels(ax,{'Initial 949','Motion','Heading','Matching','Predicted 959'});
    xtickangle(ax,20);ylabel(ax,'Signed error contribution (cm)');grid(ax,'on');
    title(ax,'Exact vector accounting in frame-959 axes');legend(ax,'Forward','Left',Location='southwest');
    ax=nexttile(layout);hold(ax,'on');
    plot(ax,inputs.time-inputs.time(1),inputs.observerLeftMps,LineWidth=1.5);
    plot(ax,inputs.time-inputs.time(1),inputs.inspvaLeftMps,LineWidth=1.5);
    xlabel(ax,'Time since frame 949 (s)');ylabel(ax,'Lateral velocity (m/s)');grid(ax,'on');
    title(ax,'Motion input versus recorded INS velocity');legend(ax,'Observer input','INS diagnostic reference',Location='northwest');
    ax=nexttile(layout);hold(ax,'on');names=unique(controls.variant,'stable');
    for k=1:numel(names)
        selected=controls.variant==names(k);
        plot(ax,controls.frame(selected),100*controls.matchedErrorM(selected),'-o',LineWidth=1.3);
    end
    grid(ax,'on');xlabel(ax,'Frame');ylabel(ax,'After-matching error (cm)');
    title(ax,{'Four recursive controls from identical frame 949','History motion unchanged in every control'});
    legend(ax,'Original','Reference lateral step','Reference full step','Exclude sign target 1300',Location='northwest');
    drawnow;exportgraphics(fig,fullfile(dest,'drift.png'),Resolution=160);
    assert(all(signs.globalTarget(signs.variant=="recorded_motion")==1300));
end
