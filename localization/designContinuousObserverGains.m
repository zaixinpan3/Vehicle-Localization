function design = designContinuousObserverGains(cfg)
% designContinuousObserverGains Alias for the current two-mode design entry.
    arguments
        cfg (1,1) struct = improvedObserverConfig()
    end
    design=designImprovedObserverGains(cfg);
end
