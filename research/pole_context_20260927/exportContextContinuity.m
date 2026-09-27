function T=exportContextContinuity()
% exportContextContinuity: Height support in continuous surrounding sectors.
    root=setupVehicleLocalization();folder=fileparts(mfilename('fullpath'));
    data=load(fullfile(root,'output','pole_miss_analysis_20260925','cases.mat'),'cases');
    previous=load(fullfile(root,'output','pole_context_20260927','development.mat'),'cache');
    T=readtable(fullfile(folder,'expanded_features.csv'));rows={};
    for k=1:numel(previous.cache)
        old=previous.cache{k};c=data.cases{find(cellfun(@(v)v.frame==old.frame,data.cases),1)};
        for j=1:numel(old.rows)
            f=old.rows{j};h=old.hypotheses(f.hypothesis);p=double(c.points);
            d=p(:,1:2)-h.axisXY-(p(:,3)-h.axisZ).*h.slopeXY;r=vecnorm(d,2,2);
            near=r<=1.25 & p(:,3)>=h.minimumZ-.25 & p(:,3)<=h.maximumZ+.25;
            p=p(near,:);d=d(near,:);r=r(near);angle=atan2(d(:,2),d(:,1));
            f=struct('frame',f.frame,'pillar',f.pillar,'hypothesis',f.hypothesis);
            for inner=[.20 .25 .35]
                for outer=[.75 1.25]
                    for sectors=[1 4 8]
                        group=floor((angle+pi)/(2*pi)*sectors)+1;group=min(group,sectors);
                        ring=r>inner & r<=outer;support=zeros(sectors,1);
                        for s=1:sectors
                            z=p(ring & group==s,3);support(s)=heightSupport(z,h.minimumZ,h.maximumZ);
                        end
                        tag=sprintf('%02d_%03d_%d',round(inner*100),round(outer*100),sectors);
                        f.(['contextHeight' tag])=sum(support)/(h.maximumZ-h.minimumZ);
                        f.(['maximumContextHeight' tag])=max(support)/(h.maximumZ-h.minimumZ);
                    end
                end
            end
            rows{end+1,1}=f; %#ok<AGROW>
        end
    end
    U=struct2table(vertcat(rows{:}));assert(isequal(T(:,{'frame','pillar','hypothesis'}),U(:,{'frame','pillar','hypothesis'})));
    T=[T U(:,4:end)];writetable(T,fullfile(folder,'context_continuity_features.csv'));
end
function total=heightSupport(z,lower,upper)
    if numel(z)<3,total=0;return;end
    [positions,~,group]=unique([z-.25;z+.25]);delta=[ones(size(z));-ones(size(z))];
    counts=cumsum(accumarray(group,delta));width=max(0,min(positions(2:end),upper)-max(positions(1:end-1),lower));
    total=sum(width(counts(1:end-1)>=3));
end
