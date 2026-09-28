function cfg=semanticPillarPrecisionConfig(spacing,profile)
% semanticPillarPrecisionConfig: Precision gates for non-pole coarse classes.
% Models score current-frame 0.6 m pillar distributions and context only.
% Pole retains its independent continuous-support distribution validator.
% Fine references enter offline training/evaluation, never online inference.
% The Downtown curb/facade operating points sacrifice substantial coverage.
% See research/semantic_precision_20260927/README.md for calibration limits.
    if nargin<1,spacing=.6;end
    if nargin<2,profile="mississippi";end
    profile=lower(string(profile));
    if profile=="missisipi",profile="mississippi";end
    cfg=struct('profile',lower(string(profile)),'enabled',abs(spacing-.6)<1e-12,'classes',["curb","trafficSign","facade"], ...
        'modelDirectory',fileparts(mfilename('fullpath')),'modelFiles',struct());
    % Mississippi recovery uses the same whole-pillar descriptors with a
    % profile-specific forest. Other profiles retain their calibrated models.
    if profile=="mississippi"
        cfg.modelFiles.curb="mississippiCurbPillarPrecisionModel.json";
    end
end
