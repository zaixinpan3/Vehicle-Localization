function T=analyzeFinalResiduals()
% analyzeFinalResiduals: Fixed-denominator range and target support diagnostics.
    root=setupVehicleLocalization();folder=fileparts(mfilename('fullpath'));
    out=fullfile(root,'output','pillar_fine_alignment_20260926');
    reference=load(fullfile(out,'reference.mat'),'references');
    result=load(fullfile(out,'final_full.mat'),'records');rows={};
    edges=[0 10 20 30 inf];counts=zeros(4,3);
    supportEdges=[0 10 30 100 inf];supportCounts=zeros(4,3);
    for k=1:1170
        r=reference.references{k};ids=result.records{k}.poleCells;
        range=vecnorm(r.points(:,1:2),2,2);covered=ismember(r.pointPillarIds,ids);
        [owners,~,group]=unique(r.pointPillarIds);n=accumarray(group,1);matched=ismember(owners,ids);
        for b=1:4
            selected=range>=edges(b) & range<edges(b+1);
            counts(b,:)=counts(b,:)+[nnz(selected),nnz(selected & covered),nnz(selected & ~covered)];
            selected=n>=supportEdges(b) & n<supportEdges(b+1);
            supportCounts(b,:)=supportCounts(b,:)+[nnz(selected),nnz(selected & matched),nnz(selected & ~matched)];
        end
    end
    for b=1:4
        rows{end+1}=struct('metric',"fine points by range (m)",'lower',edges(b),'upper',edges(b+1), ...
            'target',counts(b,1),'covered',counts(b,2),'missed',counts(b,3)); %#ok<AGROW>
        rows{end+1}=struct('metric',"pillars by fine point count",'lower',supportEdges(b),'upper',supportEdges(b+1), ...
            'target',supportCounts(b,1),'covered',supportCounts(b,2),'missed',supportCounts(b,3)); %#ok<AGROW>
    end
    T=struct2table(vertcat(rows{:}));T.coverage=T.covered./T.target;
    writetable(T,fullfile(folder,'residual_groups.csv'));disp(T);
end
