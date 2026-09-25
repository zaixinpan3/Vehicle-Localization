function plotRadialMassDistributions()
% plotRadialMassDistributions: Empirical point mass versus metric distance.
    root=setupVehicleLocalization(); folder=fullfile(root,'research','pole_distribution_20260925');
    d=load(fullfile(root,'output','pole_distribution_20260925','point_distributions.mat'));
    tables=cell(numel(d.records),1); masses=[]; labels=[];
    for k=1:numel(d.records)
        r=d.records{k}; p=double(r.points); ids=double(r.pillarIds); [occupied,~,group]=unique(ids);
        count=accumarray(group,1); h=accumarray(group,p(:,3),[],@max)-accumarray(group,p(:,3),[],@min);
        selected=count>=6 & h>=.6;
        [~,~,~,peak]=computePillarDensityCore(p,ids,r.geometry,.1,.15,.6,selected);
        radial=computePillarRadialDistribution(p,ids,r.geometry,peak,selected);
        base=ismember(occupied,r.baselineCells); fine=ismember(occupied,r.fineCells); current=ismember(occupied,r.currentCells);
        groupLabel=zeros(size(occupied)); groupLabel(base & ~fine)=2;groupLabel(current & ~base & ~fine)=3;groupLabel(fine)=1;
        include=groupLabel>0 & selected;
        fraction=radial.count./max(radial.count(:,radial.radii==.9),1);
        masses=[masses;fraction(include,:)]; labels=[labels;groupLabel(include)]; %#ok<AGROW>
        t=table(repmat(r.frame,nnz(include),1),occupied(include),groupLabel(include), ...
            radial.count(include,2)./max(radial.count(include,6),1), ...
            radial.count(include,4)./max(radial.count(include,8),1), ...
            radial.xyLinearity(include,6),radial.radialStd(include,2), ...
            'VariableNames',{'frame','pillar','group','mass15over45','mass25over75','linearity45','radialStd15'});
        tables{k}=t;
    end
    writetable(vertcat(tables{:}),fullfile(folder,'radial_reference_distributions.csv'));
    figure('Color','w','Position',[80 80 850 540]);hold on;
    names={'Fine-confirmed','Old coarse only','Current only'};colors=lines(3);
    for k=1:3
        values=masses(labels==k,:); plot(radial.radii,median(values,1),'-o','Color',colors(k,:),'LineWidth',1.8,'DisplayName',names{k});
    end
    xlabel('Radius from whole-pillar density peak [m]');ylabel('Median fraction of returns within 0.90 m');
    title('Radial mass concentration: 117-frame diagnostic reference groups');
    legend('Location','northwest');grid on;ylim([0 1]);
    exportgraphics(gcf,fullfile(folder,'radial_mass_profiles.png'),'Resolution',150);
end
