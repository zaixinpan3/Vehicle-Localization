function validatePoleSubsetCapture()
% validatePoleSubsetCapture: Match original-precision studies to end-to-end output.
    root=setupVehicleLocalization();out=fullfile(root,'output','pole_subset_20260925');
    full=load(fullfile(root,'output','coarse_lattice_20260924','subsetFull_sequence.mat'),'records');
    native=load(fullfile(out,'native.mat'),'records');reference=load(fullfile(out,'matlab.mat'),'records');
    sourceDifferences=0;backendDifferences=0;
    for k=1:numel(native.records)
        r=native.records{k};a=sort(double(r.detected(:)));b=sort(double(full.records{r.frame}.poleCells(:)));
        sourceDifferences=sourceDifferences+numel(setxor(a,b));
        backendDifferences=backendDifferences+numel(setxor(a,double(reference.records{k}.detected)));
    end
    folder=fullfile(root,'research','pole_subset_20260925');
    a=readtable(fullfile(folder,'native_pillars.csv'));b=readtable(fullfile(folder,'matlab_pillars.csv'));
    assert(isequal(a(:,{'frame','pillarIndices','found','accepted','ownCount','supportCount'}),b(:,{'frame','pillarIndices','found','accepted','ownCount','supportCount'})));
    error=max(abs(a.score-b.score));
    assert(sourceDifferences==0 && backendDifferences==0 && error<1e-9);
    report=struct('frames',numel(native.records),'sourcePrecision',"original recorded XYZ", ...
        'endToEndMaskDifferences',sourceDifferences,'backendMaskDifferences',backendDifferences,'maximumScoreError',error);
    fid=fopen(fullfile(folder,'capture_validation.json'),'w');clean=onCleanup(@()fclose(fid));
    fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));disp(report);
end
