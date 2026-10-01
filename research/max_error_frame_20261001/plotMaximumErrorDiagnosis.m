function plotMaximumErrorDiagnosis()
% plotMaximumErrorDiagnosis Render the exported diagnosis tables off screen.
% Every plotted value is available in the CSV files beside this script.
    dest=fileparts(mfilename('fullpath'));
    window=readtable(fullfile(dest,'maximum_window.csv'));budget=readtable(fullfile(dest,'error_budget.csv'),TextType='string');
    control=readtable(fullfile(dest,'closed_loop_control.csv'));summary=jsondecode(fileread(fullfile(dest,'summary.json')));
    colors=[42 120 214;235 104 52;27 175 122]/255;ink=[11 11 11]/255;muted=[137 135 129]/255;hairline=[225 224 217]/255;
    fig=figure('Visible','off','Color',[252 252 251]/255,'Position',[100 100 1300 860]);cleanup=onCleanup(@()close(fig));
    layout=tiledlayout(fig,2,2,'TileSpacing','compact','Padding','compact');
    ax=nexttile(layout);hold(ax,'on');
    series={window.lateralVelocityInputMps,window.inspvaVelocityLateralMps,window.inspvaIncrementLateralMps};
    names=["Supplied wheel/lateral motion","INSPVA reported velocity","INSPVA position increments"];
    for j=1:3,plot(ax,window.frame,series{j},'LineWidth',2,'Color',colors(j,:),'DisplayName',names(j));end
    xline(ax,summary.frame,':','Color',muted,'HandleVisibility','off');
    style(ax,'Lateral velocity in reference vehicle axes','Frame','m/s',ink,muted,hairline);legend(ax,'Location','east','Box','off','TextColor',ink);
    ax=nexttile(layout);hold(ax,'on');
    series={100*window.fusedLateralM,100*window.matchLateralM,100*window.gnssLateralM};
    names=["Fused observer","LiDAR match","GNSS at observer point"];
    for j=1:3,plot(ax,window.frame,series{j},'LineWidth',2,'Color',colors(j,:),'DisplayName',names(j));end
    xline(ax,summary.frame,':','Color',muted,'HandleVisibility','off');
    text(ax,summary.frame+1,100*summary.lateralM-1.2,sprintf('frame %d: %.1f cm',summary.frame,100*summary.lateralM),'Color',ink,'FontSize',9);
    style(ax,'Lateral discrepancy against the INSPVA trajectory','Frame','cm',ink,muted,hairline);legend(ax,'Location','southwest','Box','off','TextColor',ink);
    ax=nexttile(layout);part=budget(budget.contribution~="total" & budget.contribution~="initial",:);
    part=sortrows(part,'lateralAtMaximumM','descend');labels=["GNSS position","Heading used to rotate motion","Endpoint integration", ...
        "Longitudinal speed","Velocity-state filtering","INSPVA position vs its velocity","LiDAR matching","Lateral velocity vs INSPVA velocity"];
    keys=["gnss","heading","endpointQuadrature","longitudinal","velocityFilter","referenceKinematics","lidar","lateral"];
    [~,order]=ismember(part.contribution,keys);
    barh(ax,100*part.lateralAtMaximumM,.55,'FaceColor',colors(1,:),'EdgeColor','none');
    set(ax,'YTick',1:height(part),'YTickLabel',labels(order),'YDir','reverse');
    for j=1:height(part)
        value=100*part.lateralAtMaximumM(j);
        text(ax,min(value,0)-.25,j,sprintf('%+.2f',value),'HorizontalAlignment','right','Color',ink,'FontSize',9);
    end
    xlim(ax,[-13.5 1.5]);
    style(ax,sprintf('Propagated lateral contributions at frame %d (sum %.2f cm)',summary.frame,100*summary.lateralM),'cm','',ink,muted,hairline);
    ax=nexttile(layout);hold(ax,'on');
    plot(ax,control.frame,100*control.productionFusedLateralM,'LineWidth',2,'Color',colors(1,:),'DisplayName','Production motion');
    plot(ax,control.frame,100*control.oracleFusedLateralM,'LineWidth',2,'Color',colors(2,:),'DisplayName','Course-rotated motion (oracle)');
    xline(ax,summary.frame,':','Color',muted,'HandleVisibility','off');
    style(ax,'Fused lateral discrepancy, closed-loop rematching','Frame','cm',ink,muted,hairline);legend(ax,'Location','southwest','Box','off','TextColor',ink);
    exportgraphics(fig,fullfile(dest,'diagnosis.png'),'Resolution',160);
    exportgraphics(fig,fullfile(dest,'diagnosis.pdf'),'ContentType','vector');
end
function style(ax,name,xName,yName,ink,muted,hairline)
    title(ax,name,'Color',ink,'FontWeight','normal','FontSize',11);xlabel(ax,xName,'Color',muted);ylabel(ax,yName,'Color',muted);
    set(ax,'Color','none','XColor',muted,'YColor',muted,'GridColor',hairline,'GridAlpha',1,'Box','off','TickDir','out','FontSize',9);
    grid(ax,'on');
end
