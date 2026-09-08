function audit=auditMncavOutageCertificate(designFile,outputFile)
% auditMncavOutageCertificate Probe omitted GPS-loss/pulse-gap LMI modes.
% This is a certificate diagnostic, not a new design or a runtime override.
% In the normalized Lyapunov coordinates, the additional lidar XY channel
% contributes -2*Cl'*W*Cl. Testing W=I uses the strongest permitted bounded
% correction. A positive margin rejects this certificate, not every possible
% certificate or every nonlinear trajectory.
    loaded=load(designFile,'observerDesign','observerCfg');
    design=loaded.observerDesign;
    if isfield(design,'kind') && startsWith(string(design.kind),"aperiodic-")
        v=verifyImprovedObserverDesign(design,loaded.observerCfg);
        audit=table(["aperiodicFullPoseFlow";"timerReset"], ...
            [v.maximumFlowEigenvalue;v.maximumResetEigenvalue], ...
            [v.flowInequalityCount;nnz(design.timer.knots>=design.timer.tMin)], ...
            'VariableNames',{'mode','maximumEigenvalue','checkedCombinations'});
        writetable(audit,outputFile);disp(audit);return;
    end
    data=buildImprovedObserverCertificateData(loaded.observerCfg);
    p=design.P; y=p*design.K; schur=y*(design.X\y.');
    names=["gpsPresent","gpsAbsentMaximumLidar","allPosePulsesInactive"];
    rows=cell(3,4);
    for mode=1:3
        worst=-Inf; witness=zeros(1,3); count=0;
        for h=1:data.outputVertexCount
            output=design.N*data.outputVertices(:,:,h);
            for w=1:data.omegaVertexCount
                omega=data.omegaVertices(:,:,w); lidar=zeros(2);
                if mode==2, omega(1:2,1:2)=0; lidar=eye(2); end
                if mode==3, omega=zeros(3); end
                for f=1:data.fVertexCount
                    affine=p*(data.A+data.fVertices(:,:,f)+output)-y*omega*data.Cb;
                    matrix=affine+affine.'+design.lambda*eye(7)+schur-2*data.Cl.'*lidar*data.Cl;
                    margin=max(eig((matrix+matrix.')/2)); count=count+1;
                    if margin>worst, worst=margin; witness=[h w f]; end
                end
            end
        end
        rows(mode,:)={names(mode),worst,count,witness};
    end
    audit=cell2table(rows,'VariableNames',{'mode','worstSchurMargin','checkedCombinations','witness'});
    writetable(audit,outputFile); disp(audit);
end
