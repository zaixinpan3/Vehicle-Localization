function cfg=freshObserverTestConfig(cfg)
% Return this run's temporary design, solving changed inputs before use.
% Only the shared test fixture owns this in-memory map; no file or persistent
% variable supplies gains, and teardown discards all designs between runs.
    arguments
        cfg (1,1) struct=mncavFullObserverConfig()
    end
    name='VehicleLocalizationFreshTestGains';
    assert(isappdata(0,name),'VehicleLocalization:TestGainScope', ...
        'Use freshObserverGainsFixture before requesting test gains.');
    selection=struct();if isfield(cfg,'gainSelection'),selection=cfg.gainSelection;end
    designs=getappdata(0,name);
    key=jsonencode(struct('certificate',invariantObserverSupport.inputs(cfg),'selection',selection));
    if ~isKey(designs,key)
        [design,~]=synthesizeMncavInvariantDesign(cfg);designs(key)=design;
    end
    cfg.invariantDesign=designs(key);cfg.gainSource="fresh-lmi";
end
