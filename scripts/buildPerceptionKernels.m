function binary = buildPerceptionKernels()
% buildPerceptionKernels: Compile optional exact CPU kernels using mex -setup C++.
% Requires a supported C++ compiler, but no MATLAB Coder, GPU, or parallel pool.
% The platform-specific binary is ignored by Git; the source is authoritative.
    root = fileparts(fileparts(mfilename('fullpath')));
    source = fullfile(root, 'perception', 'native', 'perceptionKernelsMex.cpp');
    temporary = tempname;
    mkdir(temporary);
    cleanup = onCleanup(@() removeTemporaryBuild(temporary));
    buildError = [];
    % C++17 is needed for std::clamp; this is a build-local compiler option.
    options = {};
    if isunix, options = {'CXXFLAGS=$CXXFLAGS -std=c++17'}; end
    if ispc, options = {'COMPFLAGS=$COMPFLAGS /std:c++17'}; end
    try
        mex('-R2018a', '-O', options{:}, '-outdir', temporary, source);
    catch exception
        % Some Linux mex installations reject their own freshly linked ELF
        % during post-build inspection. Only this error may defer to a real
        % load-and-execute check; all compilation/link failures remain fatal.
        if ~contains(exception.message,'is not a MEX file'), rethrow(exception); end
        buildError = exception;
    end
    filename = ['perceptionKernelsMex.', mexext];
    freshBinary = fullfile(temporary, filename);
    assert(isfile(freshBinary), 'Compiled kernel binary was not created.');
    clear perceptionKernelsMex perceptionNativeAvailable
    addpath(temporary,'-begin');
    pathCleanup = onCleanup(@() rmpath(temporary));
    rehash;
    assert(strcmp(which('perceptionKernelsMex'),freshBinary), 'Smoke test must load the fresh binary.');
    mask = perceptionKernelsMex('growRoad',logical([1 1;0 1]),1, ...
        [0 0.1;0 0.2],[0 0.15 inf]);
    assert(isequal(mask,logical([1 1;0 1])), 'Native kernel smoke test failed.');
    clear perceptionKernelsMex pathCleanup
    binary = fullfile(root,'perception',filename);
    copyfile(freshBinary,binary,'f');
    rehash;
    clear perceptionNativeAvailable
    if ~isempty(buildError)
        warning('perception:MexPostBuildInspection', ...
            'mex post-build inspection reported an error; the fresh binary passed load-and-execute validation.');
    end
end

function removeTemporaryBuild(folder)
    if isfolder(folder), rmdir(folder,'s'); end
end
