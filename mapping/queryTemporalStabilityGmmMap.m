function [scores, details] = queryTemporalStabilityGmmMap(first, second, varargin)
% queryTemporalStabilityGmmMap: Query Lambda/(Lambda+kappa), with coverage.
% New maps return NaN outside owned fitting regions and a status for each
% query. details.intensity is the shared Gaussian field; omitted intensity
% and score errors are bounded by epsilon and epsilon/kappa, respectively.
    assert(numel(varargin)<=1,'Only optional class labels are accepted.');
    if isnumeric(first), points=first; map=second; else, map=first; points=second; end
    labels=strings(0,1);
    if ~isempty(varargin), labels=string(varargin{1}); end
    if isstruct(map) && isfield(map,'canonicalMap'), map=map.canonicalMap; end
    assert(isstruct(map) && isfield(map,'schemaVersion') && map.schemaVersion==2, ...
        'VehicleLocalization:UnsupportedMapSchema','Build a current repeated-observation map.');
    validateattributes(points,{'numeric'},{'real','2d','ncols',2,'finite'});
    points=double(points);
    if isfield(map,'layers'), layers=map.layers; classes=map.classLabels;
    else, layers=map; classes=string(map.classLabel); end
    perPoint=~isempty(labels) && ~isscalar(labels);
    if perPoint, assert(numel(labels)==size(points,1),'One label is required per query.'); end
    if isempty(labels), selected=classes(:); else, selected=unique(labels(:),'stable'); end
    width=numel(selected);
    if perPoint, width=1; end
    scores=nan(size(points,1),width); intensity=zeros(size(scores)); valid=false(size(scores));
    status=repmat("unknownClass",size(scores)); errorBound=zeros(size(scores)); intensityError=zeros(size(scores));
    for j=1:numel(selected)
        rows=true(size(points,1),1); column=j;
        if perPoint, rows=labels(:)==selected(j); column=1; end
        k=find(string({layers.classLabel})==selected(j),1);
        if isempty(k), continue; end
        [s,d]=evaluateField(layers(k),points(rows,:));
        scores(rows,column)=s; intensity(rows,column)=d.intensity;
        valid(rows,column)=d.valid; status(rows,column)=d.status;
        errorBound(rows,column)=d.scoreErrorBound;
        intensityError(rows,column)=layers(k).queryIndex.intensityErrorBound;
    end
    details=struct('semantics',"referenceFeatureVersusClutter",'intensity',intensity, ...
        'valid',valid,'status',status,'scoreErrorBound',errorBound, ...
        'intensityErrorBound',intensityError,'classLabels',selected);
end

function [scores,details]=evaluateField(layer,points)
    index=layer.queryIndex;
    assert(isequal(index.means,layer.componentMeans) && isequal(index.covariances,layer.componentCovariances) && ...
        isequal(index.masses,layer.componentMasses(:)) && index.intensityErrorBound==layer.params.queryIntensityTolerance, ...
        'VehicleLocalization:StaleFieldIndex','Recompile the field index after changing geometry, mass, or tolerance.');
    assert(isfinite(layer.clutterIntensity) && layer.clutterIntensity>0, ...
        'VehicleLocalization:InvalidClutter','Clutter intensity must be positive.');
    valid=false(size(points,1),1);
    for region=layer.coverageRegions.'
        valid=valid | all(points>=region(1:2).' & points<=region(3:4).',2);
    end
    [~,cells]=ismember(floor(points/index.cellSize),index.cellKeys,'rows');
    intensity=zeros(size(points,1),1);
    for j=1:size(points,1)
        candidates=index.globalCandidates;
        if cells(j)>0, candidates=unique([candidates;index.candidates{cells(j)}]); end
        for k=candidates(:).'
            covariance=layer.componentCovariances(:,:,k); delta=points(j,:)-layer.componentMeans(k,:);
            distance=delta/covariance*delta.';
            if distance<=index.radiusSquared(k)
                intensity(j)=intensity(j)+layer.componentMasses(k)/(2*pi*sqrt(det(covariance)))*exp(-0.5*distance);
            end
        end
    end
    scores=intensity./(intensity+layer.clutterIntensity); scores(~valid)=NaN;
    status=repmat("valid",size(scores)); status(~valid)="outsideCoverage";
    if layer.totalMass==0, status(valid)="noPublishedStructure"; end
    details=struct('intensity',intensity,'valid',valid,'status',status, ...
        'scoreErrorBound',min(1,index.intensityErrorBound/layer.clutterIntensity));
end
