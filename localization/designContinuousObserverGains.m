function design = designContinuousObserverGains(cfg)
% designContinuousObserverGains Continuous ISS design for the current observer.
% This alias and the numerical runtime use the same gain-design entry.
    arguments
        cfg (1,1) struct = fullObserverConfig()
    end
    design=designFullObserverGains(cfg);
end
