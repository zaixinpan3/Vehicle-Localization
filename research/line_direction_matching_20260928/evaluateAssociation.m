function evaluateAssociation()
% evaluateAssociation Check direction-compatible associations on cached sources.
    setupVehicleLocalization();addpath('output/line_direction_matching_20260928/prototype');dest=fileparts(mfilename('fullpath'));
    s=load('output/line_direction_matching_20260928/sources.mat');cfg=distributionRegistrationConfig();
    state=s.calls{1,{'predictedX','predictedY','predictedPsi'}};rows=cell(height(s.calls),10);
    for k=1:height(s.calls)
        predicted=state;if k>1,predicted=compose(state,relative(s.motion(k-1,:),s.motion(k,:)));end
        [local,~]=selectLocalProbabilityCloud(s.fixed,predicted,cfg.localMapRadius);t=tic;
        r=directionAssociationRegistration(local,s.sources{k},predicted,cfg);ms=1000*toc(t);
        event=registrationSupport.registrationPoseMeasurement(r,s.calls.timeSeconds(k));state=predicted;if ~isempty(event),state=event.pose;end
        e=state-s.calls{k,{'referenceX','referenceY','referencePsi'}};
        rows(k,:)={k,r.accepted,r.directionalAccepted,r.reason,norm(e(1:2)),rad2deg(wrap(e(3))),state(1),state(2),state(3),ms};
    end
    result=cell2table(rows,VariableNames={'frame','accepted','directional','reason','errorM','yawErrorDeg','x','y','psi','matchingMs'});
    writetable(result,fullfile(dest,'direction_association.csv'));disp(result(result.frame==601,:));fprintf('RMSE %.9f, accepted %d, maximum %.9f\n',rms(result.errorM),nnz(result.accepted),max(result.errorM));
end
function p=compose(a,b)
    r=[cos(a(3)) -sin(a(3));sin(a(3)) cos(a(3))];p=[a(1:2)+b(1:2)*r.' wrap(a(3)+b(3))];
end
function b=relative(a,c)
    r=[cos(a(3)) -sin(a(3));sin(a(3)) cos(a(3))];b=[(c(1:2)-a(1:2))*r wrap(c(3)-a(3))];
end
function x=wrap(x)
    x=atan2(sin(x),cos(x));
end
