# X-Plane Aircraft Sources

This directory contains aircraft integration notes, not installation bundles.
Install each matching X-Plane aircraft separately; the plugin provides the
actuator/sensor mapping and PX4 airframe references. See the
[airframe cards](../docs/index.md#flight-test-cards) for selection and checks.

## QuadTailsitter

QuadTailsitter uses the existing `xplane_qtailsitter` target, ID `5021` and
`[QuadTailsitter]` plugin section. Its canonical profile matches a 2 kg,
0.6 m-span, four-motor tailsitter with 10x8 propellers and no control surfaces.

The matching aircraft is supplied by its owner; no QuadTailsitter ACF is
bundled in this repository or the normal plugin package. There is one profile
and one target, not a second aircraft installation. Follow the
[QuadTailsitter card](../docs/QUADTAILSITTER_XPLANE12_TEST.md) and
[camera notes](QuadTailsitter/README.md).

TruthCapture is optional diagnostic recording. Technical validation scope is
documented in the [Developer Guide](../docs/DEVELOPER.md#verification-limits).
