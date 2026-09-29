function runLocalCurbMap()
% runLocalCurbMap Test whether long GMM axes distort local curved-road geometry.
% Only the existing offline fine-map curb observations build these surfaces.
    setupVehicleLocalization();out='output/matching_objective_20260929';dest=fileparts(mfilename('fullpath'));
    a=load('output/mississippi_mapping_calibrated/feature_observations.mat','featureData');f=a.featureData;at=string(f.featureNames)=="curb";sets=f.pointsByFeatureFrame(at,:);xyz=vertcat(sets{:});frame=cell(size(sets));
    for k=1:numel(sets),frame{k}=k*ones(size(sets{k},1),1);end
    frame=vertcat(frame{:});spacing=.2;origin=min(xyz(:,1:2),[],1);bins=floor((xyz(:,1:2)-origin)/spacing)+1;
    [keys,~,id]=unique([bins,frame],'rows');perFrame=zeros(size(keys,1),3);
    for axis=1:3,perFrame(:,axis)=accumarray(id,xyz(:,axis),[],@mean);end
    [cells,~,id]=unique(keys(:,1:2),'rows');mu=zeros(size(cells,1),3);
    for axis=1:3,mu(:,axis)=accumarray(id,perFrame(:,axis),[],@mean);end
    repeat=accumarray(id,1);index=sparse(cells(:,2),cells(:,1),(1:size(cells,1)).');
    old=load('output/mississippi_mapping_calibrated/probability_cloud.mat','cloud');base=old.cloud.components;keep=base.semanticName~="curb";oldCurbMass=sum(base.mixtureWeight(~keep));rows=cell(0,5);
    for radius=[.8 1.6]
        n=size(mu,1);covariance=zeros(2,2,n);means=mu(:,1:2);valid=false(n,1);span=ceil(radius/spacing);
        for j=1:n
            x=max(1,cells(j,1)-span):min(size(index,2),cells(j,1)+span);y=max(1,cells(j,2)-span):min(size(index,1),cells(j,2)+span);ids=nonzeros(index(y,x));ids=ids(sum((mu(ids,1:2)-mu(j,1:2)).^2,2)<=radius^2);
            if numel(ids)<4,continue;end
            p=mu(ids,1:2);center=mean(p,1);d=p-center;S=d.'*d/size(d,1);[V,E]=eig(S,'vector');[minor,i]=min(E);major=max(E);
            if major<4*max(minor,1e-5),continue;end
            normal=V(:,i);t=[-normal(2);normal(1)];means(j,:)=mu(j,1:2)-((mu(j,1:2)-center)*normal)*normal.';
            covariance(:,:,j)=max(.0025,minor)*(normal*normal.')+max(.04,major)*(t*t.');valid(j)=true;
        end
        c=struct('mean',[base.mean(keep,:);means(valid,:)],'semanticName',[base.semanticName(keep);repmat("curb",nnz(valid),1)], ...
            'covariance',cat(3,base.covariance(:,:,keep),covariance(:,:,valid)), ...
            'mixtureWeight',[base.mixtureWeight(keep);oldCurbMass*repeat(valid)/sum(repeat(valid))],'numComponents',nnz(keep)+nnz(valid));
        cloud=struct('components',c,'dimension',2,'frameCalibration',old.cloud.frameCalibration,'coordinateFrame',old.cloud.coordinateFrame, ...
            'curbGeometryModel',"localEqualScanFineMapSurfaces",'queryObservationOverlap',true);
        if isfield(old.cloud,'clockModelId'),cloud.clockModelId=old.cloud.clockModelId;end
        label="localCurb"+round(10*radius);mapFile=fullfile(out,label+"_map.mat");save(mapFile,'cloud','spacing','radius','-v7.3');
        row=replayStudy('output/root_cause_matching_20260929/finalSurface_sources.mat',label,distributionRegistrationConfig(),mapFile);
        rows(end+1,:)={radius,n,nnz(valid),row.maximumErrorM,row.rmseM}; %#ok<AGROW>
    end
    writetable(cell2table(rows,VariableNames={'radius','cells','surfaces','maximumErrorM','rmseM'}),fullfile(dest,'local_map_summary.csv'));
end
