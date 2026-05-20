# X-Plane Aircraft Sources

This directory stores aircraft assets that px4xplane packages for validation.

## QuadTailsitter

`QuadTailsitter/` is the source-controlled X-Plane 12 aircraft used with
`SYS_AUTOSTART=5021` and `config_name = QuadTailsitter`.

The model is still in hover-recovery tuning. Keep aircraft physics changes in
small, documented slices and validate them with PX4 ULog plus XPlaneTruthCapture
before using the model for transition or fixed-wing tuning.
