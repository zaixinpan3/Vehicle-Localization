function varargout = buildFineColumnShapeScores(varargin)
% buildFineColumnShapeScores: Historical column-score compatibility entry.
% Modern perception calls buildPillarShapeScores with whole-pillar evidence.
    [varargout{1:nargout}] = buildPillarShapeScores(varargin{:});
end
