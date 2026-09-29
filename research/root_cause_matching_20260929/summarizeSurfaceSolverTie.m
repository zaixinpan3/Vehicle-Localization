function summarizeSurfaceSolverTie()
% summarizeSurfaceSolverTie Export the near-degenerate real-scan solver case.
    a=load('output/root_cause_matching_20260929/surface_solver_difference.mat');j=find(abs(a.da.dy-a.db.dy)>1e-8);
    d=table(repmat(a.k,numel(j),1),a.da.pillar(j),a.da.dy(j),a.db.dy(j),a.da.explainedFraction(j),a.db.explainedFraction(j),a.da.residualStd(j),a.db.residualStd(j), ...
        VariableNames=["frame","pillar","directDy","batchedDy","directExplained","batchedExplained","directResidualStd","batchedResidualStd"]);
    writetable(d,fullfile(fileparts(mfilename('fullpath')),'surface_solver_tie.csv'));disp(d);
end
