function diagnoseOwnerSupport()
% diagnoseOwnerSupport: Separate shaft-level disagreement from owner spillover.
root=setupVehicleLocalization;folder=fullfile(root,'research','pole_geometry_20260927');rows={};summary={};
for name=["mississippi","downtown"]
 s=load(fullfile(root,'output','pole_precision_20260927',name+'.mat'),'records','references','frames');
 n=zeros(1,8);
 for k=1:numel(s.frames)
  r=s.records{k};ref=s.references{k};ids=ref.pointPillarIds;positive=unique(ids(ids>0));
  connected=[];
  for a=unique(r.hypothesisIds).'
   o=r.owners(r.hypothesisIds==a);if any(ismember(o,positive)),connected=union(connected,o);end
  end
  owners=unique(r.owners);fp=setdiff(owners,positive);n(1)=n(1)+numel(fp);n(2)=n(2)+nnz(ismember(fp,connected));
  missing=setdiff(positive,owners);allowner=[];acceptedowner=[];
  for a=1:numel(r.hypotheses)
   h=r.hypotheses(a);allowner=union(allowner,h.ownerIds);
   if h.geometryAccepted,acceptedowner=union(acceptedowner,h.ownerIds);end
  end
  reasons={intersect(missing,acceptedowner),intersect(setdiff(missing,acceptedowner),allowner),setdiff(missing,allowner)};
  for j=1:3,n(2+j)=n(2+j)+numel(reasons{j});n(5+j)=n(5+j)+nnz(ismember(ids,reasons{j}));end
  for o=owners.'
   match=find(r.owners==o);hh=r.hypotheses(r.hypothesisIds(match));valid=false;
   for a=1:numel(hh)
    h=hh(a);b=find(h.ownerIds==o);blend=min(1,max(0,(norm(h.axisXY)-15)/5));stable=h.tilt<=4+blend*2 && h.radialRms<=.10+blend*.02;
    valid=valid||((h.acceptedCount>=30||stable)&&(h.ownerCount(b)>=15||stable));
   end
   rows{end+1,1}=struct('dataset',name,'frame',s.frames(k),'pillar',o,'finePointCount',nnz(ids==o),'current',valid,'positiveShaft',ismember(o,connected)); %#ok<AGROW>
  end
 end
 summary{end+1}=struct('dataset',name,'falseOwners',n(1),'falseOwnersOnPositiveShaft',n(2),'missedOwnerGatePillars',n(3),'missedGeometryPillars',n(4),'missedProposalPillars',n(5),'missedOwnerGatePoints',n(6),'missedGeometryPoints',n(7),'missedProposalPoints',n(8)); %#ok<AGROW>
end
writetable(struct2table(vertcat(rows{:})),fullfile(folder,'broad_owner_diagnosis.csv'));
fid=fopen(fullfile(folder,'support_diagnosis.json'),'w');fprintf(fid,'%s\n',jsonencode(vertcat(summary{:}),PrettyPrint=true));fclose(fid);disp(vertcat(summary{:}));
end
