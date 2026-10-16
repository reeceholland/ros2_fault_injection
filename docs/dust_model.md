# Dust plume model: assumptions and evidence

## Status and purpose

The first plume implementation is connected to the point-cloud injector.
An active fault with `dust_model: plume` and a positive interaction coefficient
samples dust interactions along valid input rays and replaces affected surface
points with nearer returns. The original beam direction and point count are
preserved. Geometry, midpoint integration, seeded sampling, and configuration
validation and range-dependent intensity sampling are implemented; Carén-specific calibration remains future work.

Only existing valid points provide rays. Missing beams do not gain new returns.
Each sampled interaction is currently treated as a detected replacement return;
a new intensity is sampled at the replacement range when a FLOAT32 intensity
field exists. General cloud-wide intensity scaling can still run afterwards.
Background survival, multiple returns, and a calibrated sensor response are not
implemented.

Active plume faults are processed in sorted fault-ID order, after random dust.
Multiple plumes therefore mutate points sequentially; they are not integrated
as a combined density field. For an isolated demonstration, enable one plume
and leave random dust and other point mutations disabled.

Point-cloud tests cover ray direction and bounds, spatial selectivity for a
fixed seed, equal-seed output sequences, zero-coefficient forwarding, and
unchanged forwarding after deactivation. Intensity tests cover zero spread,
seeded output intensities, preservation of non-interacting points, and XYZ-only
clouds without an intensity field.

The existing random-dust mode remains the default. It selects individual points
using a probability and assigns random dust ranges. The plume mode is intended
for tests where spatially clustered disturbance matters, such as persistent
false obstacles and partial visibility loss. It is a phenomenological fault
model, not a particle-fluid or full optical sensor simulator.

## Carén reference dataset

The calibration reference is the Carén subset of
[UCHILE-Dust](https://github.com/nicolasCruzW21/UCHILE-Dust).
The authors describe an Ouster OS0 multi-echo LiDAR for all captures.
Carén uses a mobile robot with odometry in an open field/quarry, with dust
introduced by a blower. The README lists 13 recordings and 7,089 point clouds
for this subset. See the authors'
[subset description](https://github.com/nicolasCruzW21/UCHILE-Dust#subsets--statistics).

These acquisition details establish the reference conditions. They do not
establish a Gaussian physical concentration field or universal dust parameters.
The intended model is sensor-independent in structure; transferring a calibration
to another sensor or environment still requires validation.

The repository identifies the accompanying paper as *Dust filtering in LiDAR
point clouds using deep learning for mining applications*, Bruno Cavieres,
Nicolás Cruz, and Javier Ruiz-del-Solar (2025). Consult the authors'
[citation instructions](https://github.com/nicolasCruzW21/UCHILE-Dust#citation)
and cite both dataset and paper when using the data.

## Geometry assumptions

One plume is stationary and axis-aligned. Its centre and three widths are in
metres. Widths are Gaussian standard deviations, not hard plume boundaries.

For position (x, y, z), define:

```text
q_squared = ((x - center_x) / sigma_x)^2
          + ((y - center_y) / sigma_y)^2
          + ((z - center_z) / sigma_z)^2

relative_density = exp(-q_squared / 2)
```

Here `^2` means a mathematical square. Relative density is dimensionless,
equals one at the centre, and fades towards zero. It is neither a measured
mass concentration nor a spatial probability density normalised to integrate
to one. Detected dust-point density is also not automatically physical dust
concentration: beam geometry, occlusion, and detection affect what is observed.

The ray helpers accept an explicit origin and endpoint in the same frame as
the plume centre. Set `plume_frame` to a fixed frame such as `odom` to express
the centre, axes and velocity in that frame. An empty value uses the input cloud
frame. The injector looks up cloud-to-plume TF at the cloud timestamp once per
plume operation, transforms the ray origin and endpoint, and keeps output points
in the original cloud frame. Missing TF or a zero timestamp when crossing frames
skips that plume with a throttled warning; other enabled faults may still apply.
With zero velocity a plume in `odom` stays fixed as the rover moves; a plume in
the cloud frame moves with the sensor.

Rotation, wind, diffusion, source emission, and decay are outside this first
implementation. If spreading is added later, peak density must account for
dilution when modelling a source-free plume; increasing widths alone increases
the integrated density.

## Ray-interaction assumptions

A straight ray runs from the sensor origin to the original surface point.
The interaction rate at distance s is:

```text
lambda(s) = interaction_coefficient * relative_density(ray(s))
tau(L) = integral from 0 to L of lambda(s) ds
P(at least one interaction before L) = 1 - exp(-tau(L))
```

This assumes a nonhomogeneous Poisson interaction process. The coefficient is
a tunable peak rate in inverse metres, so optical depth tau is dimensionless.
The probability is an interaction probability, not a calibrated probability
of a detected LiDAR return. Intensity, detection thresholds, two-way attenuation,
multiple scattering, and background-return survival are not represented by
this helper.

The implementation approximates the integral using equal-length segments and
midpoint density samples. Segment count is `ceil(L / step_size)`, capped at
10,000; requests above that limit are rejected. The cap bounds work per ray,
not total cloud processing cost. Narrow plumes need sufficiently fine steps;
midpoint integration has no built-in error estimate and can miss narrow peaks.

## First-interaction sampling and repeatability

The caller supplies a uniform sample u in [0, 1). The helper computes:

```text
target_depth = -ln(1 - u)
```

It walks along the ray until accumulated depth crosses that threshold, then
interpolates within the segment assuming its midpoint rate is constant.
If no crossing occurs before the surface, it returns `std::nullopt`.
There is no separate probability coin flip before this sampling.

The helper owns no random generator. The point-cloud injector uses
its existing seeded generator. Matching samples, inputs, parameters, and
numerical environment makes this calculation repeatable; a seed alone does
not reproduce ROS timing or live input clouds. See
{ref}`random seed configuration <random-seeds>`.

Zero-length rays and zero coefficients produce no interaction. A uniform
sample of zero can produce an interaction at zero metres. A sensor minimum
detection range must therefore be handled explicitly by a future sensor
response layer.

## Dust-return intensity

For a sampled dust return at range r in metres, the model computes:

```text
median(r) = clamp(intercept + slope_per_m * r, minimum_median, maximum_median)
ln(intensity) ~ Normal(ln(median(r)), log_sigma^2)
```

The median is not the mean: for nonzero spread the theoretical mean is
`median(r) * exp(log_sigma^2 / 2)`. With `log_sigma: 0.0`, the model returns
exactly the median without consuming an intensity random sample. The bounds
clamp the median only; individual lognormal samples can exceed either bound.
There is no sensor saturation clamp, and extreme parameters can overflow the
numeric representation. Intensities use the input sensor's intensity scale,
not universal reflectance units.

Only points replaced by plume dust receive sampled intensities. Points without
an interaction retain their intensity unless another enabled fault changes it.
The injector modifies intensity only when a FLOAT32 intensity field exists;
XYZ-only clouds still receive spatial dust returns without gaining a new field.
Other intensity field types are left untouched by this operation.

Plume intensity uses the injector's seeded generator, shared with interaction
sampling. Identical seeds, inputs, configuration, and execution environment
reproduce the sequence. Adding an intensity field or changing from zero to
nonzero spread changes random-number consumption and can alter subsequent
interaction locations. A seed alone does not reproduce a live ROS input stream.

`dust_intensity_scale` belongs to random dust and does not control plume
intensity. `intensity_scale` is applied afterwards to the whole cloud, including
plume returns. Leave it unset or at 1.0 to inspect the sampled model directly.
The current intensity values are illustrative engineering defaults, not a fit
to Carén measurements or an Ouster OS0 calibration.

## Configuration and current defaults

These fields are registered for the point-cloud injector. They are engineering
starting values, not a fitted Carén calibration.

| Field | Default | Meaning and constraints |
| --- | --- | --- |
| `plume_frame` | empty | Frame for centre, axes and velocity; empty uses the cloud frame. |
| `dust_model` | `random` | Either `random` or `plume`; plume replaces points at sampled interaction distances. |
| `plume_center_x/y/z` | 0.0 | Finite centre coordinates in metres in `plume_frame` (cloud frame when empty). |
| `plume_sigma_x/y/z` | 1.0 | Finite, strictly positive Gaussian widths in metres. |
| `plume_interaction_coefficient` | 0.0 | Finite non-negative peak rate in inverse metres; zero means no interactions. |
| `plume_step_size` | 0.1 | Finite, strictly positive maximum integration segment length in metres. |
| `plume_intensity_median_intercept` | 30.0 | Finite intensity intercept; signed values allowed. |
| `plume_intensity_median_slope_per_m` | -2.0 | Finite intensity change per metre; signed values allowed. |
| `plume_intensity_minimum_median` | 10.0 | Finite, strictly positive lower median bound. |
| `plume_intensity_maximum_median` | 25.0 | Finite, strictly positive upper median bound, at least the minimum. |
| `plume_intensity_log_sigma` | 0.63 | Finite, non-negative standard deviation in log space; zero returns the median exactly. |

Negative centre coordinates are valid. Non-finite values, non-positive widths,
and non-positive step sizes are rejected. Parameter validation does not certify
the physical realism of a scenario or guarantee numerical accuracy.
Intensity bounds are checked together during scenario loading and runtime edits,
including defaults for omitted bounds. A rejected edit preserves the previous
configuration. To move both bounds above the current maximum, raise the maximum
first; to move both below the current minimum, lower the minimum first.

## Calibration limits and validation plan

No quantitative Carén fit is claimed by these defaults. Candidate aspect ratios,
intensity curves, and background-coexistence fits need traceable analysis before
being published as calibrated presets. Keep the following with any fitted values:

- Dataset version and exact recordings, preprocessing, labels, and return fields.
- Reproducible analysis code, units, fit domain, sample counts, and uncertainty.
- Separate recordings for fitting and evaluation to avoid neighbouring-frame leakage.
- Comparisons of spatial extent, range-dependent dust frequency, and intensity.
- Sensitivity to step size and testing on other sensors or environments.

Angular-cell coexistence of dust and background points is a proxy, not proof
of two returns from the same emitted pulse. Absolute intensity and even
dust-to-background ratios depend on sensor processing and scene composition.
Do not treat them as universal material properties.

The Gaussian field and Poisson interaction law are explicit model assumptions.
Carén is the empirical reference for evaluating simulated observations; it is
not evidence that these assumptions uniquely describe real dust.
