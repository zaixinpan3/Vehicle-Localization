function [energy, gradient] = semanticGaussianOverlap(fixed, moving, pose)
% semanticGaussianOverlap: Exact same-class integral and SE(2) derivatives.
% Integral N(x;a,A) N(x;b,B) dx = N(a;b,A+B). Covariance is
% rotated with the mean; derivatives include the rotating covariance term.
% Inputs are component structs already validated at the API boundary.
    if size(fixed.mean,2)==3
        if nargout > 1
            [energy,gradient]=spatialGaussianOverlap(fixed,moving,pose);
        else
            energy=spatialGaussianOverlap(fixed,moving,pose);
        end
        return;
    end
    c = cos(pose(3)); s = sin(pose(3));
    rotation = [c -s; s c];
    means = moving.mean*rotation.' + pose(1:2);
    meanDerivative = moving.mean*[-s -c; c -s].';
    a0 = reshape(moving.covariance(1,1,:), [], 1);
    b0 = reshape(moving.covariance(1,2,:), [], 1);
    d0 = reshape(moving.covariance(2,2,:), [], 1);
    ma = c*c*a0 - 2*c*s*b0 + s*s*d0;
    mb = c*s*(a0-d0) + (c*c-s*s)*b0;
    md = s*s*a0 + 2*c*s*b0 + c*c*d0;
    energy = 0; gradient = zeros(1,3);
    fixedActive = fixed.mixtureWeight > 0;
    movingActive = moving.mixtureWeight > 0;
    names = intersect(unique(fixed.semanticName(fixedActive)), unique(moving.semanticName(movingActive)));
    for name = names.'
        f = find(fixed.semanticName == name & fixedActive);
        allMoving = find(moving.semanticName == name & movingActive);
        fa = reshape(fixed.covariance(1,1,f), [], 1);
        fb = reshape(fixed.covariance(1,2,f), [], 1);
        fd = reshape(fixed.covariance(2,2,f), [], 1);
        % Bound temporary pair arrays for large offline maps.
        blockSize = max(1, floor(250000/max(numel(f),1)));
        for start = 1:blockSize:numel(allMoving)
            m = allMoving(start:min(start+blockSize-1,end));
            a = fa + ma(m).'; b = fb + mb(m).'; d = fd + md(m).';
            determinant = a.*d - b.*b;
            dx = fixed.mean(f,1) - means(m,1).';
            dy = fixed.mean(f,2) - means(m,2).';
            qx = (d.*dx-b.*dy)./determinant;
            qy = (a.*dy-b.*dx)./determinant;
            kernel = exp(-0.5*(dx.*qx+dy.*qy))./(2*pi*sqrt(determinant));
            weighted = kernel.*(fixed.mixtureWeight(f)*moving.mixtureWeight(m).');
            energy = energy + sum(weighted,'all');
            if nargout > 1
                da = -2*mb(m).'; db = (ma(m)-md(m)).'; dd = 2*mb(m).';
                rotationTerm = qx.*meanDerivative(m,1).' + qy.*meanDerivative(m,2).' + ...
                    0.5*(qx.^2.*da+2*qx.*qy.*db+qy.^2.*dd - ...
                    (d.*da+a.*dd-2*b.*db)./determinant);
                gradient = gradient + [sum(weighted.*qx,'all'), sum(weighted.*qy,'all'), ...
                    sum(weighted.*rotationTerm,'all')];
            end
        end
    end
end

function [energy, gradient] = spatialGaussianOverlap(fixed, moving, pose)
% spatialGaussianOverlap: Exact 3D Gaussian overlap and planar-pose gradient.
% Z origins are already aligned. Covariance derivatives include xz/yz terms.
    c=cos(pose(3)); s=sin(pose(3));
    rotation=[c -s 0;s c 0;0 0 1];
    means=moving.mean*rotation.'+[pose(1:2) 0];
    meanDerivative=moving.mean*[-s -c 0;c -s 0;0 0 0].';
    a0=reshape(moving.covariance(1,1,:),[],1);
    b0=reshape(moving.covariance(1,2,:),[],1);
    d0=reshape(moving.covariance(2,2,:),[],1);
    c0=reshape(moving.covariance(1,3,:),[],1);
    e0=reshape(moving.covariance(2,3,:),[],1);
    ma=c*c*a0-2*c*s*b0+s*s*d0;
    mb=c*s*(a0-d0)+(c*c-s*s)*b0;
    md=s*s*a0+2*c*s*b0+c*c*d0;
    mc=c*c0-s*e0; me=s*c0+c*e0;
    mf=reshape(moving.covariance(3,3,:),[],1);
    energy=0; gradient=zeros(1,3);
    fixedActive=fixed.mixtureWeight>0;
    movingActive=moving.mixtureWeight>0;
    names=intersect(unique(fixed.semanticName(fixedActive)),unique(moving.semanticName(movingActive)));
    for name=names.'
        f=find(fixed.semanticName==name & fixedActive);
        allMoving=find(moving.semanticName==name & movingActive);
        fa=reshape(fixed.covariance(1,1,f),[],1); fb=reshape(fixed.covariance(1,2,f),[],1);
        fc=reshape(fixed.covariance(1,3,f),[],1); fd=reshape(fixed.covariance(2,2,f),[],1);
        fe=reshape(fixed.covariance(2,3,f),[],1); ff=reshape(fixed.covariance(3,3,f),[],1);
        blockSize=max(1,floor(100000/max(numel(f),1)));
        for start=1:blockSize:numel(allMoving)
            m=allMoving(start:min(start+blockSize-1,end));
            a=fa+ma(m).'; b=fb+mb(m).'; cc=fc+mc(m).';
            d=fd+md(m).'; e=fe+me(m).'; h=ff+mf(m).';
            aa=d.*h-e.^2; ab=cc.*e-b.*h; ac=b.*e-cc.*d;
            ad=a.*h-cc.^2; ae=b.*cc-a.*e; af=a.*d-b.^2;
            determinant=a.*aa+b.*ab+cc.*ac;
            assert(all(determinant>0,'all'),'VehicleLocalization:InvalidCovariance','Invalid summed XYZ covariance.');
            dx=fixed.mean(f,1)-means(m,1).';
            dy=fixed.mean(f,2)-means(m,2).';
            dz=fixed.mean(f,3)-means(m,3).';
            qx=(aa.*dx+ab.*dy+ac.*dz)./determinant;
            qy=(ab.*dx+ad.*dy+ae.*dz)./determinant;
            qz=(ac.*dx+ae.*dy+af.*dz)./determinant;
            kernel=exp(-0.5*(dx.*qx+dy.*qy+dz.*qz))./((2*pi)^1.5*sqrt(determinant));
            weighted=kernel.*(fixed.mixtureWeight(f)*moving.mixtureWeight(m).');
            energy=energy+sum(weighted,'all');
            if nargout > 1
                da=-2*mb(m).'; db=(ma(m)-md(m)).'; dc=-me(m).';
                dd=2*mb(m).'; de=mc(m).';
                rotationTerm=qx.*meanDerivative(m,1).'+qy.*meanDerivative(m,2).'+ ...
                    0.5*(qx.^2.*da+2*qx.*qy.*db+2*qx.*qz.*dc+qy.^2.*dd+2*qy.*qz.*de- ...
                    (aa.*da+2*ab.*db+2*ac.*dc+ad.*dd+2*ae.*de)./determinant);
                gradient=gradient+[sum(weighted.*qx,'all'),sum(weighted.*qy,'all'),sum(weighted.*rotationTerm,'all')];
            end
        end
    end
end
