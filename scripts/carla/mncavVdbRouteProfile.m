function [path,report]=mncavVdbRouteProfile(route,cfg)
% mncavVdbRouteProfile Limit speed using both road heading and XY curvature.
% Road headings alone miss lateral lane-change offsets in sampled waypoints.
% The three-point circumcircle curvature captures these geometric corners.
% This is a scenario speed envelope, not an estimator measurement or a
% guarantee of the lateral acceleration attained by the tracking controller.
    arguments
        route table
        cfg (1,1) struct=mncavVdbConfig()
    end
    xy=[route.x_carla,route.y_carla];provided=abs(route.curvature_per_m);
    assert(size(xy,1)>=3 && all(isfinite(xy),'all') && all(isfinite(provided)), ...
        'VehicleLocalization:InvalidVdbRoute','Require finite XY waypoints and curvature.');
    keep=[true;vecnorm(diff(xy),2,2)>1e-6];xy=xy(keep,:);provided=provided(keep);
    assert(size(xy,1)>=3,'VehicleLocalization:InvalidVdbRoute','Require three distinct consecutive waypoints.');
    segment=diff(xy);s=[0;cumsum(vecnorm(segment,2,2))];
    a=segment(1:end-1,:);b=segment(2:end,:);chord=xy(3:end,:)-xy(1:end-2,:);
    denominator=vecnorm(a,2,2).*vecnorm(b,2,2).*vecnorm(chord,2,2);
    assert(all(denominator>1e-12),'VehicleLocalization:InvalidVdbRoute','Immediate path reversals are unsupported.');
    middle=2*abs(a(:,1).*b(:,2)-a(:,2).*b(:,1))./denominator;
    geometric=[middle(1);middle;middle(end)];curvature=max(provided,geometric);
    speed=min(cfg.simulation.maximumSpeedMps,sqrt(cfg.simulation.lateralAccelerationMps2./max(curvature,.001)));
    speed(end)=0;
    for k=numel(speed)-1:-1:1
        speed(k)=min(speed(k),sqrt(speed(k+1)^2+2*cfg.simulation.maximumBrakeMps2*(s(k+1)-s(k))));
    end
    path=[xy,s,speed];
    report=struct('method',"Maximum of supplied road curvature and three-point XY curvature; backward braking envelope", ...
        'keptSourceRows',find(keep),'providedCurvature',provided,'geometricCurvature',geometric, ...
        'usedCurvature',curvature,'maximumGeometricCurvature',max(geometric), ...
        'geometryLimitedWaypoints',nnz(geometric>provided+1e-8));
end
