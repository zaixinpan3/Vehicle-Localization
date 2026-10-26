# Retired localization implementations

The pose-registration localization route was retired on 2026-10-12. Its
implementation was removed from the active tree on 2026-10-26, together with
the older seven-state observers, their saved gain certificates, replay
adapters, experimental pose graph, and solver-only tests and scripts.

The removed code is recoverable from Git revision `067df04`, immediately
before the cleanup. Historical studies retain their recorded results and
revisions. Reproduce a historical implementation in a separate checkout at
the relevant revision.

Current dataset localization uses `runFullLocalizationObserver` with the
eight-state invariant design and overlap-gradient LiDAR information. See
[the current pipeline](CURRENT_PIPELINE.md) for maintained entry points.
Registration callbacks are rejected before the observer reads inputs. The
generic pose and finite-residual interfaces support synthetic validation.

`registrationSupport` contains shared mixture geometry, map projection,
view conditioning, canonicalization and finite-residual algebra. Its name
does not select a pose optimizer. `semanticGeometryConfig` and
`prepareSemanticOverlapGeometry` supply the current overlap geometry without
the removed solver, correspondence-selection or hypothesis-selection code.

`overlapGradientConfig().geometry` replaces the former registration-settings
field. Rebuild saved overlap configurations with the current factory.
`prepareMncavMotionInputs` replaces the old mixed replay exporter; it returns
motion inputs and metadata, with GNSS and LiDAR prepared by their own stages.
