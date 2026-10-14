classdef freshObserverGainsFixture < matlab.unittest.fixtures.Fixture
% One temporary gain solve per distinct configuration in this test run.
% Compatible classes share the fixture; teardown discards every result so
% the next runtests invocation must solve again, even in the same MATLAB.
    methods
        function setup(fixture)
            root=fileparts(fileparts(mfilename('fullpath')));
            addpath(root);setupVehicleLocalization();
            key='VehicleLocalizationFreshTestGains';
            assert(~isappdata(0,key),'VehicleLocalization:TestGainScope', ...
                'A fresh test-gain scope must not outlive its fixture.');
            setappdata(0,key,containers.Map('KeyType','char','ValueType','any'));
            fixture.addTeardown(@()rmappdata(0,key));
            freshObserverTestConfig();
        end
    end
    methods (Access=protected)
        function compatible=isCompatible(~,other)
            compatible=isa(other,'freshObserverGainsFixture');
        end
    end
end
