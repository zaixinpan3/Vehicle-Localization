function results = evaluateCoarseAgreement(csvFile, outputFile)
% evaluateCoarseAgreement: Apply an explicit per-channel 80 percent gate.
% Positive feature cells only: empty background cannot inflate agreement.
% Exact and one-cell Chebyshev-tolerant precision/recall are both retained.
% Tolerance is one 0.6 m grid step per axis (0.849 m at a diagonal), not a
% 0.6 m Euclidean radius. It does not make separate detected objects identical.
% The decision requires BOTH tolerant precision and recall >= 0.80; the
% stricter exact-cell decision is also reported. Neither is ground truth.
    data = readtable(csvFile);
    groups = ["all","tuning","interleaved","remaining"];
    rows = struct([]);
    for group = groups
        switch group
            case "all", selected = true(height(data),1);
            case "tuning", selected = mod(data.frame,20)==1;
            case "interleaved", selected = mod(data.frame,20)==11;
            case "remaining", selected = mod(data.frame,10)~=1;
        end
        subset = data(selected,:);
        if isempty(subset), continue; end
        for channel = ["curb","pole","trafficSign","road"]
            nb = sum(subset.(channel+"Base")); nc = sum(subset.(channel+"Cand"));
            shared = sum(subset.(channel+"Shared"));
            precision = shared/max(nc,1); recall = shared/max(nb,1);
            pt = sum(subset.(channel+"SharedTol"))/max(nc,1);
            rt = sum(subset.(channel+"RecallTol"))/max(nb,1);
            row = struct('split',group,'channel',channel,'frames',height(subset), ...
                'baselineCells',nb,'candidateCells',nc,'precision',precision, ...
                'recall',recall,'f1',2*precision*recall/max(precision+recall,eps), ...
                'precisionTol',pt,'recallTol',rt,'f1Tol',2*pt*rt/max(pt+rt,eps), ...
                'passesExact80',precision>=0.80 && recall>=0.80, ...
                'passesTolerant80',pt>=0.80 && rt>=0.80);
            rows = [rows; row]; %#ok<AGROW>
        end
    end
    results = struct2table(rows);
    if nargin>=2, writetable(results,outputFile); end
    disp(results(:,{'split','channel','precisionTol','recallTol','f1Tol','passesTolerant80'}));
end
