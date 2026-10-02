function estimate=runFullLocalizationObserver(data,lateralDesign,cfg,options)
% runFullLocalizationObserver Run the continuous ISS-designed global observer.
% The sole runtime discretizes the certified gains on the common frame clock.
% Supply aligned lateral estimates through LateralInputs. The lateralDesign
% argument remains for caller compatibility; upstream synthesis is separate.
% Missing/invalid measurements withdraw their own correction for that frame.
    arguments
        data (1,1) struct
        lateralDesign (1,1) struct %#ok<INUSA>
        cfg (1,1) struct=fullObserverConfig()
        options.LateralInputs (1,1) struct=struct()
    end
    estimate=runSynchronousLocalizationObserver(data,cfg,options.LateralInputs);
end
