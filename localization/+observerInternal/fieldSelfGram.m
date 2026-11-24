function [energy,gram]=fieldSelfGram(field)
% Squared L2 norm and pose tangent Gram of one planar Gaussian mixture.
% Independently transform two copies of the scan by poses p and q. The mixed
% derivative of their inner product at p=q=0 is <D scan,D scan>. Each
% Gaussian-pair derivative includes rotation of its mean and covariance.
    active=field.mixtureWeight>0;
    means=reshape(field.mean(active,:).',2,1,[]);
    weights=reshape(field.mixtureWeight(active),1,1,[]);
    covariances=field.covariance(:,:,active);
    spin=[0,-1;1,0];
    meanYaw=pagemtimes(spin,means);
    covarianceYaw=pagemtimes(spin,covariances)-pagemtimes(covariances,spin);
    n=numel(weights);energy=0;gram=zeros(3);
    % Bound temporary 2-by-2 pages independently of the number of components.
    for first=1:16384:n^2
        pairs=first-1:min(first+16383,n^2)-1;
        i=mod(pairs,n)+1;j=floor(pairs/n)+1;
        combined=covariances(:,:,i)+covariances(:,:,j);
        xx=combined(1,1,:);xy=combined(1,2,:);yy=combined(2,2,:);
        determinant=xx.*yy-xy.^2;
        precision=[yy,-xy;-xy,xx]./determinant;
        delta=means(:,:,i)-means(:,:,j);
        v=pagemtimes(precision,delta);
        mass=weights(:,:,i).*weights(:,:,j).*exp(-0.5*sum(delta.*v,1))./(2*pi*sqrt(determinant));
        energy=energy+sum(mass,3);

        leftShape=covarianceYaw(:,:,i);rightShape=covarianceYaw(:,:,j);
        leftShapeV=pagemtimes(leftShape,v);rightShapeV=pagemtimes(rightShape,v);
        % First yaw derivatives of the log pair kernel for its two arguments.
        leftScore=-sum(meanYaw(:,:,i).*v,1) ...
            +(sum(v.*leftShapeV,1)-sum(precision.*leftShape,[1,2]))/2;
        rightScore=sum(meanYaw(:,:,j).*v,1) ...
            +(sum(v.*rightShapeV,1)-sum(precision.*rightShape,[1,2]))/2;
        rightTangent=pagemtimes(precision,meanYaw(:,:,j)+rightShapeV);
        traceProduct=sum(pagemtimes(precision,leftShape).* ...
            pagetranspose(pagemtimes(precision,rightShape)),[1,2]);

        % D_p D_q K = K [(D_p log K)(D_q log K)' + D_p D_q log K].
        translation=precision-pagemtimes(v,'none',v,'transpose');
        cross=rightTangent-v.*rightScore;
        yaw=sum((meanYaw(:,:,i)-leftShapeV).*rightTangent,1) ...
            +traceProduct/2+leftScore.*rightScore;
        gram(1:2,1:2)=gram(1:2,1:2)+sum(mass.*translation,3);
        gram(1:2,3)=gram(1:2,3)+sum(mass.*cross,3);
        gram(3,3)=gram(3,3)+sum(mass.*yaw,3);
    end
    gram(3,1:2)=gram(1:2,3).';
end
