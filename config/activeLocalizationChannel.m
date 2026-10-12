function channel=activeLocalizationChannel(channel)
% activeLocalizationChannel Current localization uses only overlap gradients.
% The pose-registration branch was archived on 2026-10-12. Historical code
% remains available in Git and for explicitly invoked standalone diagnostics.
    arguments
        channel (1,1) string="overlapGradient"
    end
    assert(channel=="overlapGradient",'VehicleLocalization:ArchivedRegistration', ...
        'The registration localization branch is archived. Use overlapGradient; see localization/REGISTRATION_ARCHIVE.md.');
end
