function [x,names]=measureSemanticPointDistributions(ground,offGround,semantic,gc,off)
% measureSemanticPointDistributions: Raw support statistics inside whole cells.
% Continuous plane residuals and conditional side distributions do not create
% finer XY or height grids. All available owner/neighborhood returns contribute.
    semantic=string(semantic);
    if semantic=="curb"
        dims=size(ground.curbCellMask);selected=find(ground.curbCellMask);points=double(gc.groundPoints);
        [col,row]=ind2sub(dims([2 1]),double(gc.groundCellLinIdx));ids=sub2ind(dims,row,col);
        theta=double(ground.energyMaps.linearityThetaRadians);
        origin=ground.cellOrigin;spacing=ground.cellSize;
    else
        dims=offGround.columnMaps.mapSize;selected=find(offGround.(semantic+"CellMask"));
        points=double(off.points);ids=double(off.pointPillarLinIdx);
        origin=offGround.columnMaps.origin;spacing=[offGround.columnMaps.dx,offGround.columnMaps.dy];theta=zeros(dims);
    end
    names=["raw_count","raw_spanX","raw_spanY","raw_spanZ","raw_eigenMin","raw_eigenMid","raw_eigenMax", ...
        "raw_slope","raw_planeRms","raw_planeQ50","raw_planeQ90","raw_planeMax", ...
        "raw_planeAbove02","raw_planeAbove04","raw_planeAbove06","raw_zQ05","raw_zQ25","raw_zQ50","raw_zQ75","raw_zQ95", ...
        "raw_contextCount","raw_contextSlope","raw_contextRms","raw_contextEigenMin", ...
        "raw_ownerContextMean","raw_ownerContextStd","raw_ownerContextQ10","raw_ownerContextQ50","raw_ownerContextQ90", ...
        "raw_stepMax","raw_stepBestSideRms","raw_stepOwnMiddleCount","raw_stepOwnMiddleFraction","raw_stepContrast", ...
        "raw_brightCount","raw_brightFraction","raw_brightMinZ","raw_brightMaxZ","raw_brightSpanZ", ...
        "raw_intensityQ25","raw_intensityQ50","raw_intensityQ90","raw_lineNearCount","raw_lineNearFraction","raw_lineDistanceQ10"];
    x=zeros(numel(selected),numel(names));
    if isempty(selected),return;end
    groups=accumarray(ids,(1:numel(ids)).',[prod(dims),1],@(rows){rows},{zeros(0,1)});
    for i=1:numel(selected)
        id=selected(i);rows=groups{id};q=points(rows,:);n=size(q,1);if n==0,continue;end
        mu=mean(q,1);a=q-mu;cv=(a.'*a)/n;eigen=sort(max(eig(cv),0)).';span=max(q,[],1)-min(q,[],1);
        beta=(cv(1:2,1:2)+eye(2)*1e-8)\cv(1:2,3);residual=a(:,3)-a(:,1:2)*beta;
        own=[n,span,eigen,norm(beta),sqrt(mean(residual.^2)),prctile(abs(residual),[50 90]),max(abs(residual)), ...
            mean(abs(residual)>.02),mean(abs(residual)>.04),mean(abs(residual)>.06),prctile(q(:,3),[5 25 50 75 95])];
        [row,col]=ind2sub(dims,id);[cc,rr]=meshgrid(max(1,col-1):min(dims(2),col+1),max(1,row-1):min(dims(1),row+1));
        adjacent=sub2ind(dims,rr(:),cc(:));contextRows=vertcat(groups{adjacent});p=points(contextRows,:);
        center=mean(p,1);b=p-center;covar=(b.'*b)/size(p,1);fit=(covar(1:2,1:2)+eye(2)*1e-8)\covar(1:2,3);
        error=b(:,3)-b(:,1:2)*fit;owner=q(:,3)-center(3)-(q(:,1:2)-center(1:2))*fit;
        context=[size(p,1),norm(fit),sqrt(mean(error.^2)),max(0,min(eig(covar))), ...
            mean(owner),std(owner,1),prctile(owner,[10 50 90])];
        step=zeros(1,5);cellCenter=origin+([col row]-.5).*spacing;
        if semantic=="curb"
            angles=[theta(id)+pi/2+[-pi/6 0 pi/6],atan2(fit(2),fit(1))];
            for angle=angles
                normal=[cos(angle),sin(angle)];tangent=[-normal(2),normal(1)];
                t=(p(:,1:2)-cellCenter)*tangent.';v=(p(:,1:2)-cellCenter)*normal.';
                left=v<-.12 & abs(t)<.7;right=v>.12 & abs(t)<.7;
                if nnz(left)<4||nnz(right)<4,continue;end
                l=[ones(nnz(left),1),t(left)]\p(left,3);r=[ones(nnz(right),1),t(right)]\p(right,3);
                height=abs(r(1)-l(1));rms=sqrt(mean([p(left,3)-[ones(nnz(left),1),t(left)]*l; ...
                    p(right,3)-[ones(nnz(right),1),t(right)]*r].^2));
                ownT=(q(:,1:2)-cellCenter)*tangent.';
                mid=[ones(n,1),ownT]*(l+r)/2;middle=abs(q(:,3)-mid)<.07 & height>=.07 & height<=.35;
                contrast=height/max(rms,.005);
                if contrast>step(5),step=[height,rms,nnz(middle),mean(middle),contrast];end
            end
        end
        radiometry=zeros(1,8);
        if semantic~="curb" && isfield(off.pointAttributes,'intensity')
            intensity=double(off.pointAttributes.intensity(rows));bright=isfinite(intensity)&intensity>1800;
            z=q(bright,3);radiometry(1:2)=[nnz(bright),mean(bright)];
            if ~isempty(z),radiometry(3:5)=[min(z),max(z),max(z)-min(z)];end
            values=intensity(isfinite(intensity));if ~isempty(values),radiometry(6:8)=prctile(values,[25 50 90]);end
        end
        lineSupport=zeros(1,3);
        if semantic=="facade"
            lineId=offGround.facade.lineMap(id);
            if lineId>0
                line=offGround.facade.detectedLines(lineId,:);direction=line(3:4)-line(1:2);
                normal=[-direction(2),direction(1)]/max(norm(direction),eps);
                distance=abs((q(:,1:2)-line(1:2))*normal.');
                lineSupport=[nnz(distance<=.2),mean(distance<=.2),prctile(distance,10)];
            end
        end
        x(i,:)=[own,context,step,radiometry,lineSupport];
    end
    x(~isfinite(x))=0;x=floor(x*1e8+.5)/1e8;
end
