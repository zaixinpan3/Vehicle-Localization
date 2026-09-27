function checks=validatePolePillarView()
% validatePolePillarView: Exercise native camera zoom and view recovery.
% Uses public zoom calls and the same callbacks installed in the live window.
    root=setupVehicleLocalization();folder=fileparts(mfilename('fullpath'));
    visible=get(groot,'DefaultFigureVisible');set(groot,'DefaultFigureVisible','off');
    restoreVisibility=onCleanup(@()set(groot,'DefaultFigureVisible',visible));
    r=showMississippiPolePillars(900,folder);cleanup=onCleanup(@()close(r.figure));
    ax=r.axes;cloud=r.sourceScatter;z=zoom(r.figure);state=getappdata(r.figure,'PolePillarViewControls');
    assert(strcmp(z.getAxes3DPanAndZoomStyle(ax),'camera'),'Zoom must move the camera, not crop limits.');
    limits=[ax.XLim ax.YLim ax.ZLim];xyz=[cloud.XData(:) cloud.YData(:) cloud.ZData(:)];rgb=cloud.CData;
    vertices=arrayfun(@(h)h.Vertices,r.pillarPatches,'UniformOutput',false);
    aim=double(r.reference.points(1,:));direction=ax.CameraTarget-ax.CameraPosition;direction=direction/norm(direction);
    focusPolePillarView(r.figure,'pointer',[aim-1000*direction;aim+1000*direction]);
    assert(min(vecnorm(xyz-ax.CameraTarget,2,2))<1e-10,'Pointer focus must select an original return.');
    initialAngle=ax.CameraViewAngle;
    for k=1:6
        zoom(r.figure,1.5);
        assert(isequal([ax.XLim ax.YLim ax.ZLim],limits),'Magnifier zoom cropped the source limits.');
        assert(isequal([cloud.XData(:) cloud.YData(:) cloud.ZData(:)],xyz),'Zoom changed source points.');
        assert(isequal(cloud.CData,rgb),'Zoom changed source/reference colors.');
    end
    assert(ax.CameraViewAngle<initialAngle,'Camera zoom did not magnify the scene.');
    assert(isequal(arrayfun(@(h)h.Vertices,r.pillarPatches,'UniformOutput',false),vertices));
    zoomAngle=ax.CameraViewAngle;
    full=findall(r.figure,'Tag','PoleFullScene');feval(full.Callback,full,[]);
    assert(isequal(camera(ax),cameraStruct(state.home)),'Full scene did not recover its exact camera.');
    region=findall(r.figure,'Tag','PoleRegion');feval(region.Callback,region,[]);
    assert(isequal([ax.XLim ax.YLim ax.ZLim],limits),'Pole view cropped the source limits.');
    assert(norm(ax.CameraTarget-state.home.CameraTarget)>1,'Pole region did not reframe the scene.');
    % Invoke the actual live wheel wrapper and native callback, then reset.
    rotate3d(r.figure,'on');mode=getuimode(r.figure,'Exploration.Rotate3d');before=ax.CameraViewAngle;
    feval(mode.WindowScrollWheelFcn,r.figure,struct('VerticalScrollCount',-1));
    assert(ax.CameraViewAngle<before && isequal([ax.XLim ax.YLim ax.ZLim],limits));
    focusPolePillarView(r.figure,'full');
    focusPolePillarView(r.figure,'pointer',[aim-1000*direction;aim+1000*direction]);
    % Magnify the chosen source point without introducing a different picker.
    camzoom(ax,8);drawnow;
    assert(min(vecnorm(xyz-ax.CameraTarget,2,2))<1e-10);
    exportgraphics(ax,fullfile(folder,'zoomed_reference.png'),'Resolution',120);
    focusPolePillarView(r.figure,'full');
    ref=showMississippiPoleReference(500,folder);closeReference=onCleanup(@()close(ref.figure));
    assert(isscalar(findobj(ref.axes,'Type','scatter')));
    assert(ref.sourceScatter.SizeData==4 && ref.metrics.finePolePoints==296);
    assert(numel(ref.sourceScatter.XData)==65536);
    checks=struct('frame',900,'sourcePoints',size(xyz,1),'fineReferencePoints',numel(r.reference.pointIndices), ...
        'singleSourceScatter',true,'uniformMarkerSize',4,'referenceOnlyFrame',500, ...
        'referenceOnlyPoints',65536,'referenceOnlyHighlightedPoints',296, ...
        'cameraZoomStyle',z.getAxes3DPanAndZoomStyle(ax),'magnifierSteps',6, ...
        'initialCameraAngle',initialAngle,'zoomedCameraAngle',zoomAngle, ...
        'unchangedXYZ',isequal([cloud.XData(:) cloud.YData(:) cloud.ZData(:)],xyz), ...
        'unchangedRGB',isequal(cloud.CData,rgb),'unchangedGrid',true,'unchangedDataLimits',true, ...
        'pointerTargetsOriginalReturn',true,'fullSceneReset',true,'poleRegionButton',true,'wheelCallback',true, ...
        'scope','Programmatic public zoom and live callbacks; not a physical mouse-injection test');
    file=fopen(fullfile(folder,'interaction_checks.json'),'w');finish=onCleanup(@()fclose(file));
    fprintf(file,'%s\n',jsonencode(checks,PrettyPrint=true));disp(checks);
    files={fullfile(root,'scripts','showMississippiPolePillars.m'), ...
        fullfile(root,'scripts','showMississippiPoleReference.m'), ...
        fullfile(root,'scripts','focusPolePillarView.m'),fullfile(folder,'validatePolePillarView.m')};
    findings=cell(size(files));
    for k=1:numel(files),findings{k}=checkcode(files{k},'-id','-config=factory');end
    file2=fopen(fullfile(folder,'code_analysis.json'),'w');finish2=onCleanup(@()fclose(file2));
    fprintf(file2,'%s\n',jsonencode(findings,PrettyPrint=true));
    assert(all(cellfun(@isempty,findings)),'Review Code Analyzer findings.');
end

function c=camera(ax)
    c=[ax.CameraPosition ax.CameraTarget ax.CameraUpVector ax.CameraViewAngle];
end

function c=cameraStruct(s)
    c=[s.CameraPosition s.CameraTarget s.CameraUpVector s.CameraViewAngle];
end
