function compareMapShapeSemantics895()
% compareMapShapeSemantics895 Supply existing within-acquisition map shape.
% Offline isolated-frame controls. Published position covariance is retained.
    setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));out='output/frame895_curb_geometry_20261001';
    a=load(fullfile(out,'audit.mat'),'d');d=a.d;
    original=load('output/mississippi_mapping_calibrated/probability_cloud_map.mat');layers=original.probabilityCloudMap.canonicalMap.layers;
    layer=layers(string({layers.classLabel})=="curb");base=load('output/mississippi_mapping_calibrated/probability_cloud.mat');views=load(featureMapBuildConfig().probabilityCloudPath);
    variants=["published_predictive_shape","within_shape_685","within_shape_all_curbs"];
    rows=cell(0,10);results=cell(3,2);sources={d.source,d.oracle};
    for variant=1:3
        fixed=d.fixed;
        ids=[];if variant==2,ids=685;elseif variant==3,ids=find(fixed.components.semanticName=="curb").';end
        for id=ids
            baseId=views.cloud.mapConstruction.originalComponentGroups{id};assert(isscalar(baseId));
            name=erase(base.cloud.components.componentId(baseId),'curb:');component=layer.components(layer.componentIds==name);
            assert(isscalar(component));fixed.components.intrinsicCovariance(:,:,id)=component.withinBlockCovariance;
        end
        assert(isequal(fixed.components.mean,d.fixed.components.mean));assert(isequal(fixed.components.covariance,d.fixed.components.covariance));
        for mode=1:2
            r=matchLocalProbabilityCloud(fixed,sources{mode},d.initial,d.cfg);results{variant,mode}=r;
            e=r.poseXYTheta-d.ref;b=e(1:2)*rot(d.ref(3));
            rows(end+1,:)={variants(variant),mode,norm(b),b(1),b(2),rad2deg(atan2(sin(e(3)),cos(e(3)))),r.accepted,r.observableRank,r.reason,height(r.correspondences)}; %#ok<AGROW>
        end
    end
    controls=cell2table(rows,VariableNames={'variant','transportVariant','errorM','longitudinalM','lateralM','yawErrorDeg','accepted','rank','reason','correspondences'});
    writetable(controls,fullfile(dest,'map_shape_controls.csv'));disp(controls);
    save(fullfile(out,'map_shape.mat'),'controls','results');
end
function r=rot(a),r=[cos(a) -sin(a);sin(a) cos(a)];end
