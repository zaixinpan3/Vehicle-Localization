function logSum = rowLogSumExp(logValues)
% rowLogSumExp: Compute row-wise log-sum-exp values for numerically
% stable responsibility normalization and likelihood accumulation.
%
% Input:
%   logValues: [N x K] log weighted likelihood values
%
% Output:
%   logSum: [N x 1] row-wise log(sum(exp(logValues))) values
    rowMax = max(logValues, [], 2);
    logSum = -inf(size(rowMax));
    finiteMask = isfinite(rowMax);
    if any(finiteMask)
        stableValues = exp(logValues(finiteMask, :) - rowMax(finiteMask));
        logSum(finiteMask) = rowMax(finiteMask) + log(sum(stableValues, 2));
    end
end
