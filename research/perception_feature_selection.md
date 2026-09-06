# Per-invocation perception channels

September 5, 2026. This change follows the facade/sign restoration in commit
`4b35b3f769ae886c86516252d9296b2775886b87` and replaces its separate selectors.

`cfg.featureNames` is the sole modern invocation-level semantic selector.
The list is ordered and may contain any subset of:

| Channel | Mississippi default | Downtown default |
| --- | --- | --- |
| curb | Selected | Selected |
| roadMarking | Selected | Selected |
| pole | Selected | Selected |
| facade | Omitted | Selected |
| trafficSign | Selected | Selected |

```matlab
cfg = perceptionConfig("Mississippi");
% cfg.featureNames = ["curb","roadMarking","pole","trafficSign"]
cfg = perceptionConfig("Downtown");
% cfg.featureNames = ["curb","roadMarking","pole","facade","trafficSign"]
cfg.featureNames = ["facade","trafficSign"]; % This invocation only.
p = perceiveFrame(frame,cfg);
cfg.executionMode = "offline";
fine = perceiveFrame(frame,cfg);
```

Lists accept string vectors and cell arrays of names. Their order is preserved
in candidate and cloud channels; component semantic IDs index that order.
An empty string vector requests no semantic channels. Unknown names, duplicate
names and nonvector lists fail validation instead of silently changing scope.

The configuration no longer exposes `coarseProbabilityCloud.semanticNames` or
`offGroundFeatures.facadeDetectionEnabled`. Modern calls explicitly reject
those old selectors if added back, with a migration message pointing to
`featureNames`. The low-level Gaussian builder continues to receive an internal
`semanticNames` field derived from the invocation list. The explicitly historical
`legacyFull` path retains its old facade override for reference reproduction;
without an explicit override it derives facade enablement from `featureNames`.
It remains a historical full-pipeline path, not a selective execution API.

## Execution dependencies

The list selects execution as well as publication. If neither ground channel
is requested, ground-feature analysis is skipped. If no off-ground channel is
requested, off-ground feature analysis is skipped. Sign-only perception skips
pillar structural shape scores, Hough facade detection and pole detection.
Within an active ground branch, omission of road marking skips its radiometric
threshold and classification. Fine refinement visits only the requested
candidate channels.

Shared point filtering, ground segmentation, terrain preparation and the sparse
frame representation remain common preprocessing. Marking-only perception
retains the curb/road-boundary evidence used to restrict the road region; this
is a required dependency, not a published curb channel or curb point decision.
Facade exclusion can affect pole proposals when both are selected, as already
implemented in the restoration. Thus arbitrary subsets do not imply that every
class is mathematically independent of every other class.

`perception.featureNames`, `candidates.semanticNames` and the cloud channel
list expose the selection. Offline `refinement` contains exactly those
channels. Full-frame `featureMasks` retain all five logical fields for caller
compatibility, with unrequested masks false; `groundPoint` remains shared
preprocessing. An omitted channel can be distinguished from a requested channel
with zero detections by the explicit list.

The mapping configuration's `featureNames` is forwarded directly to the
perception invocation and observation collection. No separate map facade
switch remains. Its default uses the Mississippi perception profile. An all-class
map call can copy the Downtown profile list while supplying the correct MAT
file and matched pose CSV. The frame viewer accepts the same optional perception
configuration and renders only its requested channels; without a supplied
configuration it detects a Downtown filename and selects that profile.

## Validation

Before editing, both full modern profiles were captured on frames 100, 200,
300, 400 and 500 of their respective recorded sequences. On all ten frames,
the final default profiles have exactly identical fine masks, empirical XYZ
means/covariances and Gaussian mixture weights. Mississippi's previously empty
facade channel is now absent from its candidate/cloud lists, so subsequent
semantic IDs are indexed against the four-channel list. No detector threshold,
point decision, map inference equation or localization state dimension changed.

`perceptionFeatureSelectionTest` adds 42 cases: all 32 channel subsets on
Downtown frame 200 in both coarse and offline modes, exact dataset defaults,
caller ordering, invalid/duplicate names, obsolete-selector rejection,
disabled-detector parameter independence and preserved marking results on
Mississippi frame 260. The marking-only test requires nonempty reference marks.
The updated structural test retains 21 cases, including actual map-entry
selection and recorded point-level comparison. Complete affected-class test and
factory Code Analyzer outcomes (173 passes, zero failures/skips; 16 files with
zero findings) are stored under
`results/perception_feature_selection_20260905/`, alongside the ten-frame
before/after comparison. The separate in-progress observer relocation continues
to emit a setup-path warning and is outside this change. Three viewer smoke checks also passed: the
automatic Downtown profile, a facade/sign subset and an empty selection.

```matlab
setupVehicleLocalization;
setenv('VEHICLE_LOCALIZATION_DATA_ROOT',fullfile(pwd,'data'));
runtests({'tests/perceptionFeatureSelectionTest.m', ...
    'tests/structuralSemanticPerceptionTest.m','tests/pillarPerceptionTest.m', ...
    'tests/coarseSemanticProbabilityCloudTest.m','tests/coarsePerceptionPerformanceTest.m', ...
    'tests/geometricRegistrationTest.m','tests/distributionRegistrationTest.m', ...
    'tests/heightProbabilityCloudTest.m','tests/temporalStabilityGmmMapTest.m', ...
    'tests/semanticNdtGridMapTest.m','tests/pipelineRegressionTest.m'});
```

The comparison is against the previous implementation, not independent semantic
annotations. No new route-level accuracy or latency claim is made.
