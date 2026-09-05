function values = groupedFiniteMedian(groups, samples, numGroups)
% groupedFiniteMedian: Exact grouped medians for finite slope-grid statistics.
% Sorting the group/value pairs once avoids a MATLAB callback for every small
% component. Empty groups retain NaN; callers provide positive integer IDs.
    values = nan(numGroups, 1);
    if isempty(groups), return; end
    ordered = sortrows([double(groups(:)), double(samples(:))], [1 2]);
    starts = find([true; diff(ordered(:, 1)) ~= 0]);
    ends = [starts(2:end)-1; size(ordered, 1)];
    middle = (starts + ends) ./ 2;
    values(ordered(starts, 1)) = ...
        (ordered(floor(middle), 2) + ordered(ceil(middle), 2)) ./ 2;
end
