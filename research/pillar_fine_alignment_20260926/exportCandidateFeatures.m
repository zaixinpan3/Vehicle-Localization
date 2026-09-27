function T=exportCandidateFeatures(cases,assignments,filename,includeContext)
% exportCandidateFeatures: Join candidate geometry to original fine point labels.
% Fine labels are diagnostic outputs and never enter context feature extraction.
% Development frames are <=780; later frames are reserved from threshold tuning.
    root=setupVehicleLocalization();folder=fileparts(mfilename('fullpath'));addpath(folder);
    if nargin<1 || isempty(cases)
        loaded=load(fullfile(root,'output','pole_miss_analysis_20260925','cases.mat'),'cases');cases=loaded.cases;
    end
    if nargin<2 || isempty(assignments)
        loaded=load(fullfile(root,'output','pillar_shaft_speed_20260926','single_thread_evidence.mat'),'speedAssignments');
        assignments=loaded.speedAssignments;
    end
    if nargin<3 || isempty(filename),filename=fullfile(folder,'candidate_features.csv');end
    if nargin<4,includeContext=true;end
    assert(numel(cases)==numel(assignments),'Per-frame cases and evidence must agree.');
    tables=cell(numel(cases),1);frameTables=tables;
    for k=1:numel(cases)
        c=cases{k};a=assignments{k};m=c.before.columnMaps;s=m.statistics;
        ids=double(a.assigned.pillarIndices(:));mapCount=prod(c.geometry.mapSize);
        assert(isequal(ids,double(s.pillarIndices(:))),'Occupied statistical and mode indices must agree.');
        legacy=logical(c.before.poleCellMask(ids));candidate=legacy | a.assigned.found;
        fineIds=double(c.fineIds(:));valid=fineIds>=1 & fineIds<=mapCount;
        fineCounts=accumarray(fineIds(valid),1,[mapCount 1]);
        off=valid & logical(c.fineOffGround(:));fineOffCounts=accumarray(fineIds(off),1,[mapCount 1]);
        selected=union(ids(candidate),find(fineCounts));[present,which]=ismember(selected,ids);
        n=numel(selected);t=table(repmat(c.frame,n,1),selected,'VariableNames',{'frame','pillarIndex'});
        t.development=repmat(c.frame<=780,n,1);t.occupied=present;
        t.fineOwnPointCount=fineCounts(selected);t.fineOwnOffGroundPointCount=fineOffCounts(selected);
        t.fineOwner=t.fineOwnPointCount>0;t.legacy=false(n,1);t.candidate=false(n,1);
        t.legacy(present)=legacy(which(present));t.candidate(present)=candidate(which(present));
        t=copyEvidence(t,a.modes,which,present,'raw');
        t=copyEvidence(t,a.assigned,which,present,'assigned');
        mapFields={'pointScore','lineScore','blobness','coreFraction','coreHeight','coreIsolation','corePointCount','pillarZRange'};
        for f=mapFields,t.(f{1})=double(m.(f{1})(selected));end
        t.pointCount=zeros(n,1);t.pointCount(present)=s.count(which(present));
        t.range=nan(n,1);t.range(present)=vecnorm(s.meanXYZ(which(present),1:2),2,2);
        covariance=s.covarianceXYZ;
        tilt=atand(vecnorm(covariance(:,4:5)./max(covariance(:,6),eps),2,2));
        radial=sqrt(max(0,covariance(:,1)+covariance(:,3)-sum(covariance(:,4:5).^2,2)./max(covariance(:,6),eps)));
        t.wholeTilt=nan(n,1);t.wholeTilt(present)=tilt(which(present));
        t.wholeRadialStd=nan(n,1);t.wholeRadialStd(present)=radial(which(present));
        t.wholeHeightStd=nan(n,1);t.wholeHeightStd(present)=sqrt(max(0,covariance(which(present),6)));
        t.shapeProduct=t.pointScore.*t.assigned_score;
        t.contextSource=zeros(n,1);
        if includeContext
            modes=a.assigned;fallback=~modes.found & a.modes.found;
            for f=fieldnames(modes).',modes.(f{1})(fallback,:)=a.modes.(f{1})(fallback,:);end
            evaluate=ismember(ids,selected) & modes.found;
            context=measureShaftContext(c.points,c.pillarIds,c.geometry,modes,evaluate);
            assert(isequal(double(context.pillarIndices(:)),ids),'Context indices must preserve mode ordering.');
            t=copyEvidence(t,context,which,present,'context');
            sources=double(a.assigned.found)+2*double(fallback);
            t.contextSource(present)=sources(which(present));
        end
        tables{k}=t;
        frameTables{k}=table(c.frame,c.frame<=780,numel(fineIds),nnz(valid),nnz(~valid), ...
            nnz(fineCounts),sum(fineCounts(ids(candidate))),nnz(candidate), ...
            'VariableNames',{'frame','development','finePointCount','finePointCountInRoi', ...
            'finePointCountOutsideRoi','finePillarCount','coveredFinePointCount','candidatePillarCount'});
        if mod(k,20)==0,fprintf('Candidate feature export %d/%d\n',k,numel(cases));end
    end
    T=vertcat(tables{:});writetable(T,filename);
    [directory,name]=fileparts(filename);writetable(vertcat(frameTables{:}),fullfile(directory,[name,'_frames.csv']));
end

function t=copyEvidence(t,evidence,which,present,prefix)
    for field=fieldnames(evidence).'
        name=field{1};if strcmp(name,'pillarIndices'),continue;end
        value=evidence.(name);if ~isnumeric(value) && ~islogical(value),continue;end
        for column=1:size(value,2)
            key=[prefix,'_',name];if size(value,2)>1,key=[key,num2str(column)];end %#ok<AGROW>
            out=nan(height(t),1);out(present)=double(value(which(present),column));t.(key)=out;
        end
    end
end
