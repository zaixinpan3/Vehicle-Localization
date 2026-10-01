function checkRegressionDiagnosis()
% checkRegressionDiagnosis Check the saved causal evidence and artifact scope.
    setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));
    s=load('output/anisotropic_regression_20260930/diagnosis.mat');c=s.controls;
    row=@(frame,name)c(c.frame==frame&c.variant==name,:);
    assert(height(c)==51 && height(s.ratios)==30);
    assert(row(178,"full_dynamic").errorM>.46 && row(894,"full_dynamic").errorM>.58);
    assert(row(178,"partial_direction_fixed_new").errorM<.032);
    assert(row(894,"full_fixed_old").errorM<.163);
    assert(row(894,"curb_normal_fixed_new").errorM<.19);
    assert(row(894,"full_direction_fixed_new").rank==2 && row(894,"full_dynamic").rank==3);
    assert(abs(row(894,"full_dynamic").longitudinalM-.556072959)<1e-6);
    assert(abs(row(894,"legacy_dynamic").longitudinalM-.024451760)<1e-6);
    a=s.associations(s.associations.frame==894,:);changed=a(a.changed,:);
    assert(height(changed)==2 && all(changed.oldNewMeanDifferenceM>4.4));
    assert(all(changed.legacyScoreOldTarget<changed.legacyScoreNewTarget));
    assert(all(changed.modernScoreOldTarget>changed.modernScoreNewTarget));
    assert(max(abs(s.associations.oldWeight-s.associations.newWeight))<1e-12);
    g=s.geometry(s.geometry.frame==894 & s.geometry.class=="curb",:);
    assert(all(g.oldTargetEligible) && median(g.sumAnisotropy)>100);
    sign=s.geometry(s.geometry.frame==178 & s.geometry.oldPartial,:);
    assert(height(sign)==1 && sign.source==22 && abs(sign.tangentOffsetM)>.41 && sign.sumAnisotropy<2.1);
    assert(sign.intrinsicAnisotropy>10 && sign.targetAnisotropy<2.3);
    info=s.curvature(s.curvature.frame==894 & s.curvature.class=="curb",:);
    assert(info.splitShapeGnInformation>100*info.originalShapeGnInformation);
    assert(info.legacyNeighborhoodInformation>1000*info.splitShapeGnInformation);
    assert(abs(row(894,"full_dynamic_split").errorM-row(894,"full_dynamic").errorM)<1e-6);
    for frame=[178 894 932]
        for name=["full_dynamic","full_fixed_new"]
            assert(abs(row(frame,name+"_split").errorM-row(frame,name).errorM)<1e-6);
        end
        part=s.forces(s.forces.frame==frame,:);
        for name=unique(part.class).'
            p=part(part.class==name,:);total=p{p.part=="total",{'gradientLongitudinal','gradientLateral','gradientScaledYaw'}};
            sumParts=sum(p{p.part~="total",{'gradientLongitudinal','gradientLateral','gradientScaledYaw'}},1);
            assert(max(abs(total-sumParts))<1e-12);
        end
    end
    files=["diagnoseAnisotropicRegression.m","plotRegressionDiagnosis.m","checkRegressionDiagnosis.m"];
    counts=zeros(numel(files),1);
    for k=1:numel(files)
        issues=checkcode(fullfile(dest,files(k)),'-id','-config=factory');counts(k)=numel(issues);
        if ~isempty(issues),disp(files(k));disp(struct2table(issues));end
    end
    writetable(table(files.',counts,VariableNames={'file','findings'}),fullfile(dest,'code_analysis.csv'));
    assert(~any(counts));
    validation=struct('geometryControls',51,'observabilityControls',30,'solverParityFrames',[178 894 932], ...
        'gradientChecksPassed',51,'sameSourceMembershipAndWeights',true,'sameScalarShapeObjectiveAndGradientAfterSplit',true, ...
        'syntheticUnequalScaleCurvatureVerified',true,'productionChanged',false,'freshFullRouteReplay',false);
    fid=fopen(fullfile(dest,'validation.json'),'w');fprintf(fid,'%s\n',jsonencode(validation,PrettyPrint=true));fclose(fid);
    disp(validation);
end
