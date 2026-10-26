function channel=activeLocalizationChannel(channel)
% activeLocalizationChannel Current localization uses only overlap gradients.
% The retired pose-registration implementation is recoverable from Git only.
    arguments
        channel (1,1) string="overlapGradient"
    end
    assert(channel=="overlapGradient",'VehicleLocalization:ArchivedRegistration', ...
        'The registration localization branch is removed. Use overlapGradient; see localization/CURRENT_PIPELINE.md.');
end
