function checks=validateFeaturePillarView()
% validateFeaturePillarView: Check multi-class rendering and camera caches.
    root=setupVehicleLocalization();folder=fullfile(fileparts(mfilename('fullpath')),'multifeature');
    visible=get(groot,'DefaultFigureVisible');set(groot,'DefaultFigureVisible','off');
    restore=onCleanup(@()set(groot,'DefaultFigureVisible',visible));
    r=showMississippiFeaturePillars(500,folder);finish=onCleanup(@()close(r.figure));
    cloud=r.sourceScatter;ax=r.axes;xyz=[cloud.XData(:),cloud.YData(:),cloud.ZData(:)];rgb=cloud.CData;
    limits=[ax.XLim ax.YLim ax.ZLim];state=getappdata(r.figure,'PolePillarViewControls');
    vertices=arrayfun(@(h)h.Vertices,r.pillarPatches,'UniformOutput',false);
    counts=arrayfun(@(name)r.metrics.features.(name).referencePointCount,r.featureNames);
    selected=arrayfun(@(name)r.metrics.features.(name).selectedPillarCount,r.featureNames);
    assert(isequal(counts,[296 507 64]) && isequal(selected,[2 12 81]));
    assert(r.metrics.sharedCoarseCells==2 && r.metrics.overlappingReferencePoints==0);
    for k=1:numel(r.featureNames)
        f=r.metrics.features.(r.featureNames(k));
        assert(nnz(all(rgb==f.colorRGB,2))==f.referencePointCount);
        channel=r.perception.candidates.semanticNames==r.featureNames(k);
        assert(isequal(sort(r.patchPillarIds(r.patchClasses==k)), ...
            sort(double(r.perception.candidates.pillarIndices{channel}(:)))));
    end
    for k=1:6,zoom(r.figure,1.5);end
    rotate3d(r.figure,'on');mode=getuimode(r.figure,'Exploration.Rotate3d');
    feval(mode.WindowScrollWheelFcn,r.figure,struct('VerticalScrollCount',-1));
    assert(isequal(cloud.CData,rgb) && isequal(cloud.ColorData,rgb));
    assert(isequal([ax.XLim ax.YLim ax.ZLim],limits));
    assert(isequal([cloud.XData(:),cloud.YData(:),cloud.ZData(:)],xyz));
    assert(isequal(arrayfun(@(h)h.Vertices,r.pillarPatches,'UniformOutput',false),vertices));
    button=findall(r.figure,'Tag','PoleRegion');feval(button.Callback,button,[]);
    assert(isequal([ax.XLim ax.YLim ax.ZLim],limits));
    focusPolePillarView(r.figure,'full');
    assert(isequal(ax.CameraPosition,state.home.CameraPosition) && isequal(ax.CameraTarget,state.home.CameraTarget));
    checks=struct('frame',500,'sourcePoints',size(xyz,1),'referencePointCounts',counts, ...
        'selectedCellCounts',selected,'sharedCells',2,'singleSourceScatter',isscalar(findobj(ax,'Type','scatter')), ...
        'uniformMarkerSize',cloud.SizeData,'sourceXYZPreserved',true,'colorCachesPreserved',true, ...
        'gridMembershipPreserved',true,'zoomAndWheelCallbacksPassed',true,'featureRegionAndResetPassed',true);
    paths={fullfile(root,'scripts','showMississippiFeaturePillars.m'), ...
        fullfile(root,'scripts','showMississippiPolePillars.m'),mfilename('fullpath')};
    findings=cell(size(paths));
    for k=1:numel(paths),findings{k}=checkcode(paths{k},'-id','-config=factory');end
    checks.codeAnalyzer=findings;
    file=fopen(fullfile(folder,'validation.json'),'w');closeFile=onCleanup(@()fclose(file));
    fprintf(file,'%s\n',jsonencode(checks,PrettyPrint=true));disp(checks);
    assert(all(cellfun(@isempty,findings)),'Review Code Analyzer findings.');
end
