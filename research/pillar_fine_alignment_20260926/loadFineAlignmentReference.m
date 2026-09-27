function [pointIndices,metadata] = loadFineAlignmentReference(frames,root)
% loadFineAlignmentReference: Load frozen original-frame Mississippi labels.
% Returned cells follow the requested frame order. No perception is rerun and
% no configuration or point mask is changed. The source files were admitted
% by the recorded hash/overlap audit; this loader performs structural checks.
% Examples: refs=loadFineAlignmentReference(1:1170); refs{1} are frame-1 IDs.
    if nargin<2 || isempty(root),root=setupVehicleLocalization();end
    if nargin<1 || isempty(frames),frames=1:1170;end
    frames=double(frames(:));
    assert(all(isfinite(frames) & frames==floor(frames) & frames>=1 & frames<=1170), ...
        'Reference frames must be integers from 1 through 1170.');
    persistent cache cacheRoot
    if isempty(cache) || ~strcmp(cacheRoot,root)
        references=cell(1170,1);seen=false(1170,1);
        for worker=1:4
            filename=fullfile(root,'output','fine_matching_20260919',sprintf('inputs_%d.mat',worker));
            part=load(filename,'frames','selectedIndices','counts','cfg','processed');
            sourceFrames=double(part.frames(:));
            assert(part.processed==numel(sourceFrames),'A fine reference partition is incomplete.');
            names=string(part.cfg.featureNames(:));pole=find(names=="pole");
            assert(isscalar(pole) && size(part.selectedIndices,1)==numel(sourceFrames) && ...
                size(part.selectedIndices,2)==numel(names),'Invalid fine reference channel layout.');
            assert(all(sourceFrames>=1 & sourceFrames<=1170 & sourceFrames==floor(sourceFrames)) && ...
                ~any(seen(sourceFrames)) && numel(unique(sourceFrames))==numel(sourceFrames), ...
                'Fine reference partitions overlap or contain invalid frames.');
            for k=1:numel(sourceFrames)
                indices=double(part.selectedIndices{k,pole}(:));
                assert(numel(indices)==part.counts(k,pole) && ...
                    all(isfinite(indices) & indices>=1 & indices==floor(indices)) && ...
                    numel(unique(indices))==numel(indices),'Invalid original-frame fine indices.');
                references{sourceFrames(k)}=indices;
            end
            seen(sourceFrames)=true;
        end
        assert(all(seen),'Frozen fine references must cover all 1170 Mississippi frames.');
        cache=references;cacheRoot=root;
    end
    pointIndices=cache(frames);
    metadata=struct('dataset',"Mississippi",'frames',frames, ...
        'sourceRevision',"48d043b083ccc2fb44c19726655223b1d5054484", ...
        'originalLatticeRevision',"796c737344e6c29c3e7aa8869aab2630dc0ebbb1", ...
        'referenceKind',"Frozen offline 0.3 m fine pole indices in original raw-frame order", ...
        'pointCount',sum(cellfun(@numel,pointIndices)), ...
        'totalAvailableFrames',1170,'runtimeReferenceLookupAllowed',false);
end
