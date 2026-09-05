function [energy, gradient] = semanticGaussianOverlap(fixed, moving, pose)
% semanticGaussianOverlap: Exact same-class integral and SE(2) derivatives.
% Integral N(x;a,A) N(x;b,B) dx = N(a;b,A+B). Covariance is
% rotated with the mean; derivatives include the rotating covariance term.
% Inputs are component structs already validated at the API boundary.
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
    names = intersect(unique(fixed.semanticName), unique(moving.semanticName));
    for name = names.'
        f = find(fixed.semanticName == name);
        allMoving = find(moving.semanticName == name);
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
