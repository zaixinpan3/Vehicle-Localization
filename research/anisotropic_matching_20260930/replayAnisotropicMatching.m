function summary=replayAnisotropicMatching(label,cfg)
% replayAnisotropicMatching Recursive replay with frozen, raw-verified inputs.
% Reference poses score outputs only. Source windows and independent odometry
% come from the September 29 production replay; perception is unchanged.
    setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));
    if nargin<2,cfg=anisotropicRegistrationConfig();end
    if isfield(cfg,'prototype')
        shadow=fullfile(dest,'prototype');addpath(shadow,'-begin');
        restore=onCleanup(@()rmpath(shadow)); %#ok<NASGU>
        clear prepareSemanticRegistrationGeometry gaussianRegistrationResiduals;
        assert(strcmp(which('prepareSemanticRegistrationGeometry'),fullfile(shadow,'prepareSemanticRegistrationGeometry.m')));
    end
    out='output/anisotropic_matching_20260930';
    input=load('output/source_shape_matching_20260929/shape50.mat','sources');
    old=load('output/line_direction_matching_20260928/production/report.mat','report');calls=old.report.calls;
    odom=load('output/line_direction_matching_20260928/sources.mat','motion');motion=odom.motion;
    mc=featureMapBuildConfig();map=load(mc.probabilityCloudPath,'cloud');
    n=height(calls);state=calls{1,{'predictedX','predictedY','predictedPsi'}};
    rows=cell(n,11);maximum=[];
    for k=1:n
        predicted=state;
        if k>1
            a=motion(k-1,:);b=motion(k,:);r=rotation(a(3));
            relative=[(b(1:2)-a(1:2))*r,wrap(b(3)-a(3))];
            stateRotation=rotation(state(3));
            predicted=[state(1:2)+relative(1:2)*stateRotation.',wrap(state(3)+relative(3))];
        end
        result=matchLocalProbabilityCloud(map.cloud,input.sources{k},predicted,cfg);
        event=registrationSupport.registrationPoseMeasurement(result,calls.timeSeconds(k));
        state=predicted;if ~isempty(event),state=event.pose;end
        reference=calls{k,{'referenceX','referenceY','referencePsi'}};e=state-reference;
        rows(k,:)={k,result.accepted,result.directionalAccepted,result.reason,norm(e(1:2)), ...
            rad2deg(wrap(e(3))),state(1),state(2),state(3),1000*result.matchingSeconds,result.observableRank};
        if k>1&&(isempty(maximum)||norm(e(1:2))>maximum.errorM)
            maximum=struct('frame',k,'errorM',norm(e(1:2)),'result',result,'source',input.sources{k}, ...
                'predicted',predicted,'reference',reference);
        end
        if mod(k,200)==0,fprintf('%s replay %d/%d\n',label,k,n);end
    end
    replay=cell2table(rows,VariableNames={'frame','accepted','directional','reason','errorM','yawErrorDeg','x','y','psi','matchingMs','rank'});
    writetable(replay,fullfile(dest,label+".csv"));
    save(fullfile(out,label+".mat"),'replay','maximum','cfg','-v7.3');
    summary=table(string(label),maximum.frame,maximum.errorM,rms(replay.errorM),prctile(replay.errorM,95), ...
        nnz(replay.accepted),nnz(replay.directional),median(replay.matchingMs),replay.errorM(178),replay.errorM(932), ...
        VariableNames={'variant','maximumFrame','maximumErrorM','rmseM','p95M','full','directional','medianMatchingMs','frame178ErrorM','frame932ErrorM'});
    writetable(summary,fullfile(dest,label+"_summary.csv"));disp(summary);
end
function r=rotation(a),r=[cos(a) -sin(a);sin(a) cos(a)];end
function a=wrap(a),a=atan2(sin(a),cos(a));end
