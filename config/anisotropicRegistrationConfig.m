function cfg=anisotropicRegistrationConfig()
% anisotropicRegistrationConfig Experimental continuous Gaussian SE(2) matcher.
% Every class uses both complete covariances, including their rotation, with
% no point/line switch. The September 30 Mississippi replay has higher maximum
% error than the validated default; this mode is explicitly opt-in.
    cfg=distributionRegistrationConfig();cfg.method="anisotropicD2D";
    cfg=rmfield(cfg,{'partialSign','lineDirection','support'});
    cfg.geometric=rmfield(cfg.geometric,'minimumLineAnisotropy');
end
