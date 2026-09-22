function trace_solver()
% trace_solver Instrument a temporary copy and verify unchanged solver output.
% The generated diagnostic copy lives under output, never on the runtime path.
    dest=fileparts(mfilename('fullpath'));out=fullfile(pwd,'output/frame959_matching_diagnosis_20260922');
    active=fileread(which('registerSemanticProbabilityCloud'));
    instrumented=replace(active,'function result = registerSemanticProbabilityCloud(', ...
        'function [result,trace] = tracedRegistration959(');
    instrumented=replace(instrumented,'q=zeros(3,1); converged=false;', ...
        'q=zeros(3,1); converged=false;trace=cell(0,1);');
    marker="if iteration==1, result.initialSimilarity=system.similarity; end";
    assert(contains(instrumented,marker));
    instrumented=replace(instrumented,marker,marker+newline+ ...
        "trace{end+1}=struct('iteration',iteration,'q',q,'system',system);");
    fid=fopen(fullfile(out,'tracedRegistration959.m'),'w');assert(fid>=0);fprintf(fid,'%s',instrumented);fclose(fid);
    addpath(out);cleanup=onCleanup(@()rmpath(out));
    s=load(fullfile(out,'diagnostic.mat'));
    [result,trace]=tracedRegistration959(s.fixed,s.source,s.seed,s.cfg.registration);
    assert(max(abs(result.poseXYTheta-s.results{1}.poseXYTheta))<1e-10);
    rows=cell(numel(trace),11);pairs=cell(numel(trace),1);
    for k=1:numel(trace)
        v=trace{k};pose=s.seed+(v.q.*[1;1;1/s.cfg.registration.yawLeverArm]).';
        e=pose-s.reference;classCount=zeros(1,3);
        classes=["curb","pole","trafficSign"];
        for j=1:3,classCount(j)=nnz(v.system.pairs.semanticName==classes(j));end
        rows(k,:)={k,norm(e(1:2)),rad2deg(e(3)),v.system.similarity,v.system.cost, ...
            v.system.numPairs,classCount(1),classCount(2),classCount(3),pose(1),pose(2)};
        p=struct2table(v.system.pairs);p.iteration=repmat(k,height(p),1);pairs{k}=p;
    end
    iterations=cell2table(rows,VariableNames={'iteration','errorM','yawErrorDeg','similarity','frozenCost','pairs', ...
        'curbPairs','polePairs','signPairs','x','y'});
    writetable(iterations,fullfile(dest,'iterations.csv'));writetable(vertcat(pairs{:}),fullfile(dest,'iteration_pairs.csv'));
    save(fullfile(out,'iteration_trace.mat'),'trace','result');disp(iterations);
end
