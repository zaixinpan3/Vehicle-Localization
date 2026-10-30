classdef observerGainSelectionSupport
% Engineering selection among fresh solutions of the original certificate.
% No reference trajectory, hard coupling bound or new stability assumption.
    methods (Static)
        function d=select(cfg)
            g=cfg.gainSelection;validateattributes(g.enabled,{'logical'},{'scalar'});
            validateattributes(g.penaltyWeights,{'double'},{'real','finite','nonnegative','vector','nonempty'});
            weights=unique([0,g.penaltyWeights(:).']);best=Inf;d=[];
            rows=struct('weight',{},'accepted',{},'noiseProxy',{},'couplingRatio',{},'failure',{});
            for weight=weights
                row=struct('weight',weight,'accepted',false,'noiseProxy',Inf,'couplingRatio',NaN,'failure',"");
                try
                    candidate=invariantObserverSupport.synthesizeCandidate(cfg,weight);
                    check=invariantObserverSupport.verify(candidate,cfg);
                    assert(check.certified,'VehicleLocalization:CandidateCertificate','Candidate failed independent verification.');
                    score=candidate.meanSquareBound;
                    assert(isfinite(score) && score>=0,'VehicleLocalization:CandidateScore','Require a finite noise proxy.');
                    row.accepted=true;row.noiseProxy=score;
                    row.couplingRatio=observerGainSelectionSupport.couplingRatio(candidate.P,cfg);
                    if score<best,best=score;d=candidate;end
                catch exception
                    if ~ismember(string(exception.identifier),["VehicleLocalization:InfeasibleInvariantDesign", ...
                            "VehicleLocalization:CommonLmiNumericalFailure","VehicleLocalization:CommonLmiIterationLimit", ...
                            "VehicleLocalization:MotionResponseSynthesis","VehicleLocalization:CandidateCertificate"])
                        rethrow(exception);
                    end
                    row.failure=string(exception.identifier)+": "+string(exception.message);
                end
                rows(end+1)=row; %#ok<AGROW>
                fprintf('GAIN CANDIDATE weight %g: accepted %d, final noise %.8g, coupling ratio %.6g\n', ...
                    weight,row.accepted,row.noiseProxy,row.couplingRatio);
            end
            assert(~isempty(d),'VehicleLocalization:GainSelectionFailed','No candidate passed the original certificate.');
            d.gainSelection=struct('method',"soft-objective-search-original-feasible-set", ...
                'candidates',rows,'selectedWeight',d.selectionObjective.penaltyWeight, ...
                'criterion',"minimum final primitive-noise proxy among certified candidates", ...
                'certificateFeasibleSetChanged',false,'referenceUsed',false);
            fprintf('GAIN SELECTED weight %g; final noise %.8g\n',d.gainSelection.selectedWeight,best);
        end

        function ratio=couplingRatio(P,cfg)
        % Diagnostic score only; no pass/fail threshold belongs to the proof.
            z=cfg.invariantObserver;n=z.noise;
            a=P(1,2:3)/P(1,1).*[n.velocityMps/z.theta,n.accelerationMps2/z.theta^2];
            ratio=norm(a)/n.lidarPositionM;
        end
    end
end
