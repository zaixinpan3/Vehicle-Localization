function replayMetricScreens()
% replayMetricScreens Reproduce rejected alternatives in a local path shadow.
% Production defaults and map files are not changed. The prototype directory
% must never be installed in the application's regular MATLAB path.
    setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));
    labels=["full_covariance","power2","power3","point_soft_only", ...
        "point_soft_power2","neighborhood4","neighborhood2","orientation_overlap","orientation_power2"];
    powers=[1 2 3 1 2 1 1 1 2];radii=[0 0 0 0 0 4 2 0 0];
    for k=1:numel(labels)
        previous=[];
        if isfile(fullfile(dest,labels(k)+".csv")),previous=readtable(fullfile(dest,labels(k)+".csv"));end
        cfg=anisotropicRegistrationConfig();
        cfg.prototype=struct('shapeMode',"full",'softAllClasses',k<=3);
        if k>=8,cfg.prototype.shapeMode="orientation";end
        cfg.geometric.anisotropyExponent=powers(k);
        if radii(k)>0,cfg.geometric.neighborhoodRadius=radii(k);end
        replayAnisotropicMatching(labels(k),cfg);
        repeated=readtable(fullfile(dest,labels(k)+".csv"));
        if ~isempty(previous) && labels(k)~="power2"
            difference=max(abs(repeated{:,{'x','y','psi'}}-previous{:,{'x','y','psi'}}),[],'all');
            assert(difference<1e-7 && isequal(repeated.reason,previous.reason));
            fprintf('%s prototype parity %.12g\n',labels(k),difference);
        else
            % Initial power2 poses were overwritten by the failed path-shadow
            % verification. Check its separately retained original metrics.
            assert(labels(k)=="power2");
            assert(abs(max(repeated.errorM(2:end))-.460452876563158)<1e-7);
            assert(abs(repeated.errorM(178)-.232864848268989)<1e-7);
            assert(abs(repeated.errorM(932)-.178490054104678)<1e-7);
            fprintf('power2 original saved metric parity passed\n');
        end
    end
end
