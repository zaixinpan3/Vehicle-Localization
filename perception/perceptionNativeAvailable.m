function available = perceptionNativeAvailable(backend)
% perceptionNativeAvailable: Resolve optional compiled CPU raster kernels.
% Only compatible binary availability is cached. Rebuilding clears this function; no
% frame, classification result, or timing result is cached.
    if nargin < 1, backend = "auto"; end
    backend = string(backend);
    assert(isscalar(backend) && any(backend == ["auto", "matlab", "native"]), ...
        'perception:InvalidBackend', 'executionBackend must be auto, matlab, or native.');
    if backend == "matlab", available = false; return; end
    persistent installed
    if isempty(installed)
        installed = false;
        if exist('perceptionKernelsMex','file') == 3
            installed = isequal(perceptionKernelsMex('version'),7);
        end
    end
    available = installed;
    assert(backend ~= "native" || available, 'perception:NativeUnavailable', ...
        'Run buildPerceptionKernels to compile the current native backend.');
end
