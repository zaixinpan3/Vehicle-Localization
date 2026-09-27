function T=captureFineAlignmentBaseline()
% captureFineAlignmentBaseline: Compare frozen coarse masks with original fine points.
    root=setupVehicleLocalization();folder=fileparts(mfilename('fullpath'));
    out=fullfile(root,'output','pillar_fine_alignment_20260926');
    if ~isfolder(out),mkdir(out);end
    frames=1:1170;[pointIndices,metadata]=loadFineAlignmentReference(frames,root);
    previous=load(fullfile(root,'output','pillar_shaft_20260926','full_pipeline.mat'),'records','g');
    source=matfile(fullfile(root,'data','raw','MissisipiPointClouds.mat'));
    rows=cell(1170,1);references=rows;
    for first=1:50:1170
        indices=first:min(first+49,1170);block=source.pointClouds(1,indices);
        for k=indices
            frame=block(k-first+1);
            [m,d]=measureFinePoleAlignment(frame,pointIndices{k},previous.records{k}.poleCells,previous.g);
            m.frame=k;rows{k}=m;
            references{k}=struct('frame',k,'pointIndices',pointIndices{k}, ...
                'points',double([frame.x(pointIndices{k}),frame.y(pointIndices{k}),frame.z(pointIndices{k})]), ...
                'pointPillarIds',d.finePointPillarIds,'finePillarIds',d.finePillarIds);
        end
    end
    T=struct2table(vertcat(rows{:}));T=movevars(T,'frame','Before',1);
    writetable(T,fullfile(folder,'baseline_frames.csv'));
    geometry=previous.g;
    save(fullfile(out,'reference.mat'),'references','geometry','metadata','T','-v7.3');
    disp(varfun(@sum,T,'InputVariables',{'finePointCount','finePointCountInRoi', ...
        'coveredFinePointCount','finePillarCount','coveredFinePillarCount', ...
        'candidatePillarCount','extraPillarCount'}));
end
