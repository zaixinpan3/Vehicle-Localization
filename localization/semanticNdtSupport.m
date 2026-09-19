classdef semanticNdtSupport
% semanticNdtSupport Soft Gaussian-mixture registration with stored map mass.
% Map mixtureWeight enters once, without renormalization or repeatability.
% Source class-balanced mass prevents scan count from setting pose confidence.
    methods (Static)
        function model=prepare(fixed,moving,cfg)
            names=intersect(unique(fixed.semanticName),unique(moving.semanticName));
            groups=cell(numel(names),1);
            for k=1:numel(names)
                fi=find(fixed.semanticName==names(k) & fixed.mixtureWeight>0);
                mi=find(moving.semanticName==names(k) & moving.mixtureWeight>0);
                g=struct('name',names(k),'target',fi,'source',mi, ...
                    'fm',fixed.mean(fi,:),'fc',fixed.covariance(:,:,fi), ...
                    'fw',fixed.mixtureWeight(fi),'mm',moving.mean(mi,:), ...
                    'mc',moving.covariance(:,:,mi),'sw',moving.mixtureWeight(mi));
                g.sw=g.sw/max(sum(g.sw),eps)/max(1,numel(names));
                groups{k}=g;
            end
            model=struct('groups',{groups},'cfg',cfg,'sourceCount',moving.numComponents);
        end

        function [cost,gradient,details]=evaluate(model,pose)
        % Exact derivative of -sum_i alpha_i sum_j pi_j K_ij.
        % K_ij is the Gaussian product integral; covariance rotates with yaw.
            theta=pose(3);c=cos(theta);s=sin(theta);r=[c -s;s c];
            nc=numel(model.groups);
            classCost=zeros(nc,1);classGradient=zeros(3,nc);similarity=0;
            sources=[];targets=[];posteriorValues=[];pairNames=strings(0,1);
            matched=0;noise=model.cfg.noiseStandardDeviation^2;
            for index=1:nc
                g=model.groups{index};
                if isempty(g.source)||isempty(g.target),continue;end
                mean=g.mm(:,1:2)*r.'+pose(1:2);
                derivative=g.mm(:,1:2)*[-s -c;c -s].';
                rotated=pagemtimes(pagemtimes(r,g.mc(1:2,1:2,:)),r.');
                % Bound temporary arrays without changing component weights.
                blockSize=max(1,floor(65536/numel(g.target)));
                for first=1:blockSize:numel(g.source)
                    idx=first:min(first+blockSize-1,numel(g.source));
                    a=reshape(g.fc(1,1,:),[],1)+reshape(rotated(1,1,idx),1,[])+noise;
                    b=reshape(g.fc(1,2,:),[],1)+reshape(rotated(1,2,idx),1,[]);
                    d=reshape(g.fc(2,2,:),[],1)+reshape(rotated(2,2,idx),1,[])+noise;
                    determinant=a.*d-b.^2;
                    dx=g.fm(:,1)-mean(idx,1).';dy=g.fm(:,2)-mean(idx,2).';
                    qx=(d.*dx-b.*dy)./determinant;qy=(a.*dy-b.*dx)./determinant;
                    kernel=exp(-.5*(dx.*qx+dy.*qy))./(2*pi*sqrt(determinant));
                    if size(g.fm,2)==3
                        % Independent conditional height evidence is retained
                        % in the full spatial Gaussian product, not a Z state.
                        [kernel,kgradient]=semanticNdtSupport.spatialKernel(g,idx,pose,noise,model.cfg);
                    else
                        da=-2*reshape(rotated(1,2,idx),1,[]);
                        db=reshape(rotated(1,1,idx)-rotated(2,2,idx),1,[]);dd=-da;
                        turn=qx.*derivative(idx,1).'+qy.*derivative(idx,2).'+ ...
                            .5*(qx.^2.*da+2*qx.*qy.*db+qy.^2.*dd- ...
                            (d.*da+a.*dd-2*b.*db)./determinant);
                        kgradient=cat(3,qx,qy,turn);
                    end
                    weighted=g.fw.*kernel;intensity=sum(weighted,1);
                    likelihood=model.cfg.backgroundDensity+intensity;
                    alpha=g.sw(idx).';posterior=weighted./likelihood;
                    classCost(index)=classCost(index)-sum(alpha.*intensity);
                    for axis=1:3
                        classGradient(axis,index)=classGradient(axis,index)- ...
                            sum(weighted.*alpha.*kgradient(:,:,axis),'all');
                    end
                    inlier=intensity./likelihood;
                    similarity=similarity+sum(alpha.*inlier);
                    matched=matched+nnz(inlier>=model.cfg.minimumInlierProbability);
                    if nargout>2
                        [~,best]=max(posterior,[],1);
                        keep=inlier>=model.cfg.minimumInlierProbability;
                        sources=[sources;g.source(idx(keep))]; %#ok<AGROW>
                        targets=[targets;g.target(best(keep))]; %#ok<AGROW>
                        posteriorValues=[posteriorValues;inlier(keep).']; %#ok<AGROW>
                        pairNames=[pairNames;repmat(g.name,nnz(keep),1)]; %#ok<AGROW>
                    end
                end
            end
            cost=sum(classCost);gradient=sum(classGradient,2);
            if nargout>2
                details=struct('classCost',classCost,'classGradient',classGradient, ...
                    'similarity',similarity,'matchedSources',matched, ...
                    'correspondences',table(sources,targets,pairNames,posteriorValues, ...
                    'VariableNames',{'source','target','semanticName','inlierProbability'}));
            end
        end

        function [hessian,classHessian]=curvature(model,pose)
        % Differentiate the exact analytic gradient in physical coordinates.
        % This includes soft-assignment ambiguity and rotating covariance.
            hessian=zeros(3);classHessian=zeros(3,3,numel(model.groups));
            steps=[1e-4,1e-4,1e-5];
            for k=1:3
                delta=zeros(1,3);delta(k)=steps(k);
                [~,gp,dp]=semanticNdtSupport.evaluate(model,pose+delta);
                [~,gm,dm]=semanticNdtSupport.evaluate(model,pose-delta);
                hessian(:,k)=(gp-gm)/(2*steps(k));
                classHessian(:,k,:)=reshape((dp.classGradient-dm.classGradient)/(2*steps(k)),3,1,[]);
            end
            hessian=(hessian+hessian.')/2;
            classHessian=(classHessian+permute(classHessian,[2 1 3]))/2;
        end

        function [kernel,derivative]=spatialKernel(g,idx,pose,noise,cfg)
        % Full XYZ convolution; only X/Y/yaw are optimized.
            theta=pose(3);r=[cos(theta) -sin(theta) 0;sin(theta) cos(theta) 0;0 0 1];
            dr=[-sin(theta) -cos(theta) 0;cos(theta) -sin(theta) 0;0 0 0];
            kernel=zeros(numel(g.target),numel(idx));derivative=zeros(numel(g.target),numel(idx),3);
            for column=1:numel(idx)
                j=idx(column);mu=g.mm(j,:)*r.'+[pose(1:2) 0];dm=g.mm(j,:)*dr.';
                cm=r*g.mc(:,:,j)*r.';dc=dr*g.mc(:,:,j)*r.'+r*g.mc(:,:,j)*dr.';
                for i=1:numel(g.target)
                    covariance=g.fc(:,:,i)+cm+diag([noise noise cfg.heightNoiseStandardDeviation^2]);
                    delta=g.fm(i,:)-mu;v=covariance\delta.';
                    kernel(i,column)=exp(-.5*delta*v)/sqrt((2*pi)^3*det(covariance));
                    derivative(i,column,:)=[v(1),v(2),dm*v+.5*(v.'*dc*v-trace(covariance\dc))];
                end
            end
        end
    end
end
