function shaftSpeedUseBackend(originalPath, baselineDir, useBaseline)
% shaftSpeedUseBackend: Switch frozen sources and MEX outside timed regions.
    path(originalPath);
    if useBaseline, addpath(baselineDir,'-begin'); end
    clear perceptionKernelsMex perceptionNativeAvailable structuralPillarConfig ...
        pillarShaftConfig analyzeStructuralPillars detectPolePillars ...
        findPillarShaftModes assignPillarShaftSupport
    rehash;
    resolved = string(which('perceptionKernelsMex'));
    if useBaseline
        assert(startsWith(resolved,string(baselineDir)+filesep), ...
            'The frozen baseline MEX did not resolve from the baseline folder.');
    else
        assert(~startsWith(resolved,string(baselineDir)+filesep), ...
            'The optimized run still resolves the baseline MEX.');
    end
    assert(perceptionNativeAvailable("native"));
end
