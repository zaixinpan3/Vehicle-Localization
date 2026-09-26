function benchmarkShaftPipeline()
% benchmarkShaftPipeline: Paired timings and output identity after optimization.
    root=setupVehicleLocalization();folder=fileparts(mfilename('fullpath'));frames=1:10:1170;
    source=matfile(fullfile(root,'data','raw','MissisipiPointClouds.mat'));block=source.pointClouds(1,frames);
    saved=load(fullfile(root,'output','pillar_shaft_20260926','full_pipeline.mat'));
    base=perceptionConfig();base.offGroundFeatures.pole.detector="pillar";base.offGroundFeatures.pole.probabilityEvidence="distribution";
    sub=base;sub.offGroundFeatures.pole.detector="subset";sub.offGroundFeatures.pole.probabilityEvidence="subset";
    next=base;next.offGroundFeatures.pole.detector="shaft";next.offGroundFeatures.pole.probabilityEvidence="shaft";
    configurations={base,sub,next};elapsed=zeros(numel(frames),3);
    for warmup=1:3,for j=1:3,perceiveFrame(block(1),configurations{j});end,end
    for k=1:numel(frames)
        order=circshift(1:3,mod(k,3));
        for j=order
            timer=tic;p=perceiveFrame(block(k),configurations{j});elapsed(k,j)=toc(timer);
            if j==3
                expected=saved.records{frames(k)};
                actual=double(p.candidates.pillarIndices{p.candidates.semanticNames=="pole"});
                assert(isequal(actual,expected.poleCells),'Optimized kernel changed a full-capture mask.');
                for field=fieldnames(expected.components).'
                    assert(isequaln(p.probabilityCloud.components.(field{1}),expected.components.(field{1})),'Optimized kernel changed Gaussian output.');
                end
            end
        end
    end
    T=table(frames(:),1000*elapsed(:,1),1000*elapsed(:,2),1000*elapsed(:,3), ...
        'VariableNames',{'frame','legacyMs','subsetMs','shaftMs'});
    writetable(T,fullfile(folder,'paired_timing.csv'));disp(median(T{:,2:4}));
end
