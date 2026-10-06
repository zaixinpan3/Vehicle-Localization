classdef perceptionViewerInteractionTest < matlab.unittest.TestCase
% perceptionViewerInteractionTest: Native pcshow identity and saved-view repair.
    methods (TestClassSetup)
        function paths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(root));
            setupVehicleLocalization;
        end
    end
    methods (Test)
        function previewKeepsPointRotationTargetDiscoverable(testCase)
            file=fullfile(fileparts(fileparts(mfilename('fullpath'))),'data','raw','MissisipiPointClouds.mat');
            testCase.assumeTrue(isfile(file),'Recorded source data are required.');
            preview=showMississippiPerception(91,file);
            cleanup=onCleanup(@() close(preview.figure));
            cloud=findobj(preview.figure,'Type','scatter','Tag','pcviewer');
            testCase.verifyNumElements(cloud,1);
            verifyRotationMenu(testCase,preview.figure);
            testCase.verifyNumElements(cloud.XData,nnz(isfinite(preview.frame.x)));
            testCase.verifyEqual(getappdata(cloud,'OriginalFramePointIndices'), ...
                find(all(isfinite([preview.frame.x(:),preview.frame.y(:),preview.frame.z(:)]),2)));
        end
        function restorePreservesViewColorsAndOriginalPointTips(testCase)
            fig=makeViewer();
            cleanup=onCleanup(@() closeIfValid(fig));
            ax=ancestor(findobj(fig,'Tag','pcviewer'),'axes');
            campos(ax,[12 -8 6]);camtarget(ax,[1 2 0]);
            before=cameraAndCloud(fig);
            folder=testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            file=fullfile(folder.Folder,'viewer.fig');savefig(fig,file);close(fig);
            restored=openfig(file,'invisible');
            restoredCleanup=onCleanup(@() close(restored));
            restorePerceptionFigure(restored);
            testCase.verifyEqual(cameraAndCloud(restored),before);
            cloud=findobj(restored,'Tag','pcviewer');
            testCase.verifyEqual(string(rotate3d(restored).Enable),"on");
            verifyRotationMenu(testCase,restored);
            tip=perceptionPointTip([],struct('Target',cloud,'DataIndex',2));
            testCase.verifyTrue(any(contains(string(tip),'Original index: 2')));
            testCase.verifyTrue(any(contains(string(tip),'Layer: pole')));
            frame=struct('x',[2;3;4],'y',[0;1;0],'z',[0;2;3]);
            source=getappdata(restored,'PointTipSource');
            updatePerceptionDisplay(restored,frame,source.featureMasks,source.featureNames,8,12);
            after=cameraAndCloud(restored);
            testCase.verifyEqual(after.camera,before.camera);
            testCase.verifyEqual(after.tag,'pcviewer');
            testCase.verifyEqual(after.counter,'Frame 8 / 12');
            testCase.verifyEqual(cloud.PointCloud.Location,after.xyz);
            testCase.verifyEqual(cloud.ColorData,after.rgb);
            testCase.verifyEqual(string(ancestor(cloud,'axes').PCUserData.colorMapData),"userspecified");
            % Repeated restoration must bind one live menu, not accumulate it.
            restorePerceptionFigure(restored);
            verifyRotationMenu(testCase,restored);
        end
        function regionalViewsKeepOriginalPointsAndStableOrbit(testCase)
            fig=makeViewer();cleanup=onCleanup(@() closeIfValid(fig));
            cloud=findobj(fig,'Tag','pcviewer');ax=ancestor(cloud,'axes');
            xyz=[cloud.XData(:),cloud.YData(:),cloud.ZData(:)];
            indices=getappdata(cloud,'OriginalFramePointIndices');
            boxes={[-5 5;-5 5;-2 5],[.8 1.2;.8 1.2;1.8 2.2]};
            for k=1:numel(boxes)
                framePerceptionView(fig,boxes{k});
                testCase.verifyEqual(ax.CameraTarget,mean(boxes{k},2).','AbsTol',1e-12);
                testCase.verifyEqual(ax.Projection,'orthographic');
                testCase.verifyEqual(string(ax.Clipping),"on");
                testCase.verifyFalse(ax.PCUserData.rotateFromCenter);
                testCase.verifyGreaterThanOrEqual(ax.XLim(2),max(xyz(:,1)));
                testCase.verifyLessThanOrEqual(ax.XLim(1),min(xyz(:,1)));
                testCase.verifyEqual(ax.PCUserData.dataLimits,[ax.XLim ax.YLim ax.ZLim]);
                for angle=0:45:315
                    pointclouds.internal.pcui.rotateAxes(ax,45,10,ax.CameraTarget,'z','up');drawnow;
                    testCase.verifyTrue(all(isfinite([ax.CameraPosition ax.CameraTarget ax.CameraUpVector])));
                    testCase.verifyEqual(ax.CameraTarget,mean(boxes{k},2).','AbsTol',1e-10);
                    [a,b,c]=ndgrid(boxes{k}(1,:),boxes{k}(2,:),boxes{k}(3,:));
                    offsets=[a(:),b(:),c(:)]-ax.CameraTarget;
                    forward=ax.CameraTarget-ax.CameraPosition;distance=norm(forward);forward=forward/distance;
                    right=cross(forward,ax.CameraUpVector);right=right/norm(right);
                    up=cross(right,forward);height=distance*tand(ax.CameraViewAngle/2);
                    pixels=getpixelposition(ax);width=height*pixels(3)/pixels(4);
                    testCase.verifyLessThanOrEqual(max(abs(offsets*right.')),width+1e-10);
                    testCase.verifyLessThanOrEqual(max(abs(offsets*up.')),height+1e-10);
                    testCase.verifyEqual([cloud.XData(:),cloud.YData(:),cloud.ZData(:)],xyz);
                    testCase.verifyEqual(getappdata(cloud,'OriginalFramePointIndices'),indices);
                end
            end
        end
        function refreshPreservesNativeAutomaticGeometry(testCase)
            fig=makeViewer();cleanup=onCleanup(@() closeIfValid(fig));
            cloud=findobj(fig,'Tag','pcviewer');ax=ancestor(cloud,'axes');
            ax.PlotBoxAspectRatioMode='auto';
            source=getappdata(fig,'PointTipSource');
            frame=struct('x',[0;1;2],'y',[0;1;0],'z',[0;2;3]);
            rotate3d(fig,'on');
            updatePerceptionDisplay(fig,frame,source.featureMasks,source.featureNames,8,12);
            testCase.verifyEqual(ax.PlotBoxAspectRatioMode,'auto');
            testCase.verifyEqual(string(rotate3d(fig).Enable),"on");
            testCase.verifyEqual(cloud.PointCloud.Location,[frame.x frame.y frame.z]);
        end
        function pointTipsPrintSourceIndexAfterFiniteFiltering(testCase)
            fig=figure('Visible','off');cleanup=onCleanup(@() closeIfValid(fig));ax=axes(fig);
            frame=struct('x',[0 NaN;1 2],'y',[0 NaN;1 0],'z',[0 NaN;2 3]);
            xyz=[frame.x(:),frame.y(:),frame.z(:)];finite=all(isfinite(xyz),2);
            pcshow(xyz(finite,:),'Parent',ax);masks=struct('pole',logical([0 0;1 0]));
            setappdata(fig,'PerceptionDataset',"Downtown");
            updatePerceptionDisplay(fig,frame,masks,"pole",416,500);
            cloud=findobj(fig,'Tag','pcviewer');
            output=evalc("perceptionPointTip([],struct('Target',cloud,'DataIndex',3));");
            testCase.verifyTrue(contains(output,'[Downtown frame 416] OriginalIndex=4'));
            testCase.verifyTrue(contains(output,'row=2 col=2'));
            testCase.verifyEqual(evalin('base','lastPickedPoint.originalIndex'),4);
            testCase.verifyEqual(evalin('base','pickedPointIndices'),4);
        end
    end
end

function verifyRotationMenu(testCase,fig)
    rotate3d(fig,'on');
    mode=getuimode(fig,'Exploration.Rotate3D');
    menu=findall(mode.UIContextMenu,'Tag','contextPCRotationCenter');
    testCase.assertNumElements(menu,1,'Rotate 3D must own the point-cloud menu.');
    testCase.verifyEqual(mode.UIContextMenu.Tag,'PCRotateContextMenu');
    testCase.verifyNumElements(findall(fig,'Tag','contextPCRotationCenter'),1);
    ax=ancestor(findobj(fig,'Tag','pcviewer'),'axes');
    original=ax.PCUserData.rotateFromCenter;
    callback=menu.Callback;
    feval(callback{1},menu,[],callback{2:end});
    testCase.verifyEqual(ax.PCUserData.rotateFromCenter,~original);
    feval(callback{1},menu,[],callback{2:end});
    testCase.verifyEqual(ax.PCUserData.rotateFromCenter,original);
    datacursormode(fig,'on');rotate3d(fig,'on');
    testCase.verifyEqual(getuimode(fig,'Exploration.Rotate3D').UIContextMenu,menu.Parent);
end

function fig=makeViewer()
    fig=figure('Visible','off');ax=axes(fig);
    frame=struct('x',[0;1;2],'y',[0;1;0],'z',[0;2;3]);
    pcshow([frame.x frame.y frame.z],'Parent',ax,'MarkerSize',8);
    masks=struct('pole',[false;true;false]);
    updatePerceptionDisplay(fig,frame,masks,"pole",7,12);
end

function result=cameraAndCloud(fig)
    cloud=findobj(fig,'Type','scatter');ax=ancestor(cloud,'axes');
    result=struct('camera',[ax.CameraPosition ax.CameraTarget ax.CameraUpVector ax.CameraViewAngle], ...
        'xyz',[cloud.XData(:) cloud.YData(:) cloud.ZData(:)],'rgb',cloud.CData, ...
        'tag',cloud.Tag,'markerSize',cloud.SizeData,'counter',findall(fig,'Tag','PerceptionFrameCounter').String);
end

function closeIfValid(fig)
    if isgraphics(fig),close(fig);end
end
