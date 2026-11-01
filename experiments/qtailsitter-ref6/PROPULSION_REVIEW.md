# Ref6 independent propulsion review

Historical 6 kg design review, retained for the reproducible propeller BEM
method only. The CURRENT compact revision uses a 2 kg mass and 500 W shaft
cap per motor, not the 750 W assumption below. Geometry, hover and vehicle
inertia numbers below are historical, not the current aircraft specification.
Use design.json, sizing.json and REPORT.md for current assumptions and limits.

2026-11-01. Bounded analytical review only. No ACF, repository, plugin,
installation, or controller changes made. No X-Plane test performed by this
reviewer. These are test candidates, not a calibrated X-Plane model or a
flight-readiness finding.

## Recommendation

For a simple two-blade, 10-inch-diameter, constant-geometric-8-inch-pitch,
linearly tapered NACA 2412 candidate, start with **2.60 times the baseline
root and tip chords**, preserving their ratio:

| Quantity | Candidate |
| --- | ---: |
| Root chord | 0.101833334 ft = 1.222 in = 31.039 mm |
| Tip chord | 0.049833334 ft = 0.598 in = 15.189 mm |
| Radius | 0.416666667 ft = 0.127 m |
| Blade count | 2 |
| Geometric pitch | 8 in = 0.2032 m |

This is a power-matched surrogate, NOT recovered APC blade geometry. The
baseline chords are 0.039166667/0.019166667 ft, with three blades and
0.409999996 ft nominal radius. Scaling by 1.5 merely preserves approximate
blade-count-weighted chord and is insufficient for this target.

Choose power matching over exact thrust matching: the calculation needs
2.616x chord to absorb 577.465 W at 12,000 rpm, so 2.60x rounds down slightly.
Matching 26.271 N instead needs about 2.881x chord and predicts 641 W there,
or approximately 815 W at 13,000 rpm. A single chord multiplier cannot match
both APC thrust and power with this prescribed taper, twist, and polar.

Constant pitch means beta(r) = atan(0.2032 / (2*pi*r)), not constant blade
angle. At r/R = 0.25, 0.50, 0.75, 0.95, beta is approximately 45.53,
26.99, 18.75, 15.01 degrees. Use physical radial stations, including any hub
offset. Do not assume the ACF incidence-array indices are those stations.
The baseline caps its inner incidences at 45 degrees; retaining that cap
is a geometry change from the calculation and requires rechecking. If the
candidate is later implemented, check root/tip AND stored rib chords,
incidences, actual radius/radius modifiers, all four props, and saved
Plane Maker interpretation for consistency. Do not edit only one redundant
geometry representation and assume the simulator uses it.

## Calculation

Inputs read: aircraft/QuadTailsitter/QuadTailsitter.acf and its supplied
Airfoil/NACA 2412 (popular).afl. The latter contains four tabulated polars.
The principal calculation deliberately uses the first, higher-drag block
(header begins 0.10000), interpolating Cl and Cd linearly in alpha. It does
not claim to reproduce X-Plane's Reynolds interpolation or aerodynamic
implementation.

Static annular blade-element/momentum model, SI units:

    rho = 1.225 kg/m^3; R = 0.127 m; B = 2
    active radius = 0.2R ... R; 160 midpoint annuli
    c(r) = k * [0.039166667 - 0.020000000*(r/R)] * 0.3048
    Omega = rpm*pi/30
    Vt = Omega*r - u; W^2 = Vt^2 + vi^2
    phi = atan2(vi, Vt); alpha = beta - phi
    Cn = Cl*cos(phi) - Cd*sin(phi)
    Ctan = Cl*sin(phi) + Cd*cos(phi)
    dT/dr = B*rho*W^2*c*Cn/2 = 4*pi*rho*r*vi^2*F
    dQ/dr = B*rho*W^2*c*Ctan*r/2 = 4*pi*rho*r^2*vi*u*F
    F = (2/pi)*acos(exp[-B*(R-r)/(2*r*sin(phi))])
    P = Omega * integral(dQ/dr dr)

Axial induction and swirl are solved together: u = vi*Ctan/Cn and
vi = Omega*r*tan(phi)/(1 + (Ctan/Cn)*tan(phi)). Scan phi from near zero
through beta+2.4 degrees for sign changes in the thrust residual and
bisect each root. Principal result uses the largest-phi root; the
smallest-phi alternative is also checked. This is a Prandtl tip-loss
approximation, with an unloaded inner disk, no separate hub-loss factor,
no compressibility correction, no airframe interference or ground effect.

| Static per prop, 2.60x chord | Thrust | Shaft power |
| --- | ---: | ---: |
| 12,000 rpm | 23.95 N | 573.45 W |
| 13,000 rpm | 28.11 N | 729.09 W |
| 6 kg hover, axes canted 5 degrees | 14.77 N | 277.54 W |

Hover speed is 9,422 rpm, shaft torque 0.281 N m, using constant-coefficient
T proportional to rpm^2 and P proportional to rpm^3 scaling. This scaling
omits changing Reynolds number. Four-prop hover shaft power is 1.11 kW;
electrical demand is higher and needs motor/ESC efficiency and battery sag.
The recommendation assumes **750 W mechanical shaft output per motor**.
If 750 W is electrical input, reduce the allowable shaft power accordingly.

## Independent anchor and limits

[APC primary performance table](https://www.apcprop.com/files/PER3_10x8E.dat)
reports 26.271 N / 577.465 W at 12,000 rpm static, and 30.923 N /
734.980 W at 13,000 rpm static. Its 13,000 rpm power reaches 886.735 W at
44.77 mph axial inflow. Thus 13,000 rpm plus 750 W is not an achievable
operating point everywhere: the motor must slow under increased load.
At 12,000 rpm APC's table peaks near 695 W, leaving more power margin.

[APC's method description](https://www.apcprop.com/technical-information/engineering/)
states these are computational estimates using actual geometry and vortex
analysis, not measured thrust-stand data. The supplied 12k point is verified
against that table, not independently validated experimentally.

APC-based coefficient scaling predicts roughly 9,000 rpm / 243 W per motor
for the same canted 6 kg hover. The difference from this candidate is
expected: different geometry and section assumptions. Do not force agreement
by adjusting both motor power and prop chord simultaneously.

The baseline user-reported 12.4 N / 217 W / 13,400 rpm gives approximately
Ct=0.0520, Cp=0.0163 using nominal 9.84-inch diameter and sea-level density.
The supplied APC 12k point gives Ct about 0.1288, Cp about 0.0557.
These diagnose why retaining the baseline narrow blades is inappropriate;
they are not a transferable simulator correction factor. Also, 4*12.4 N
is slightly below 5.103*g even before cant, so treat those rounded baseline
numbers as an approximate operating observation, not a force-balanced datum.

## Uncertainty

- At 12k, the selected calculation has alpha around 36 degrees at 0.20R,
  21 degrees at 0.50R, 11 degrees at 0.70R and 8 degrees at 0.90R.
  The inner blade is stalled. It is not a high-fidelity attached-flow design.
- Local section Reynolds numbers are about 48,000 to 163,000 at 12k and
  lower in hover. The first polar block is only an approximation there;
  especially the inner blade needs lower-Re data. Do not substitute the
  high-Re polar merely because it gives more thrust.
- Using each other supplied polar wholesale yields 25.58, 27.12, 27.58 N
  and 562, 568, 566 W at 12k. These are sensitivity checks, not valid local
  Reynolds solutions or statistical confidence bounds.
- Perturbing Cl by -15% and Cd by +50% yields 20.13 N / 571 W at 12k;
  Cl +15%, Cd -20% yields 27.43 N / 625 W. At 13k the latter scales to
  about 795 W, exceeding the rating. The nominal 21 W static headroom is
  NOT a conservative guaranteed margin.
- 80/160/320-annulus results are 23.987/23.954/23.954 N and
  574.09/573.45/573.38 W. Multiple roots occur in four of 160 annuli;
  choosing the smallest-phi roots gives 23.72 N / 571.86 W. Small numerical
  error does not eliminate model error or stall-branch uncertainty.
- Reserve at least approximately 20% thrust uncertainty and 20% power
  uncertainty for initial engineering screening, not as a probability bound.
  Actual installation, density, inflow and X-Plane behavior can fall outside
  it. Initially validate around 12k rather than demanding loaded 13k.

## Loaded Response And Inertia

Use J_total*dOmega/dt = Qmotor - Qprop. J_total includes blades, hub,
motor rotor and any reflected drivetrain inertia. A no-load RPM step or an
ACF throttle-time field alone does not measure loaded thrust response.
The baseline contains _throt_time_prop=0.1 s and
_spool_time_compressor_prop=5 s; this review does not establish which fields
govern the electric model. Log actual response before assigning meaning.

Provisional physical budget: aim to investigate J_total = 5e-5 to 1e-4
kg m^2 per assembly; test 1.5e-4 and 2e-4 as slow-response cases. These
are design assumptions, not recovered ACF or APC inertia values. Integrating
an illustrative solid NACA 2412 section with area approximately 0.0822*c^2,
the selected taper, 0.2R hub cutoff, and density 1100-1500 kg/m^3 gives
8.8-11.9 g blade material and J_blades = 4.6e-5 to 6.2e-5 kg m^2.
Hub/rotor mass is extra; actual molded or hollow construction can differ.
Measure mass distribution or use a pendulum/inertia measurement before
calling any of these numbers identified. At fixed radius and thickness
ratio, blade inertia scales approximately B*c^2: this candidate's blade
contribution is roughly (2/3)*2.6^2 = 4.51 times baseline, before the small
radius change, so preserving baseline response is not justified.

At hover Omega about 987 rad/s and Qprop about 0.281 N m:

- Locally constant motor torque: tau_Omega = J*Omega/(2*Qprop), about
  0.088-0.175 s for the preferred J range.
- Locally constant motor shaft power: tau_Omega = J*Omega/(3*Qprop),
  about 0.058-0.117 s. An ESC speed loop changes these dynamics again.
- Equivalent first-order 10-90% rise is 2.20*tau, roughly 0.13-0.39 s
  before extra command, ESC and inflow dynamics. This is a local estimate,
  not an exact finite-step thrust result; thrust varies as Omega^2.
- A 10% RPM increase in 0.10 s requires average inertial torque
  J*(0.1*Omega)/0.10 = 0.049-0.099 N m above aerodynamic torque.
  At the final RPM, Qprop is about 0.340 N m; check torque/current limits,
  not just a 750 W nameplate. RPM-down steps also need braking assessment.
- Near rated RPM there is almost no static power reserve and forward
  inflow can consume it entirely. Do not extrapolate hover response to
  full throttle. Never use P/Omega as a startup torque law near zero RPM.

Vehicle inertia is separate: mass, wing area and span do not determine it.
Use component inertia tensors plus parallel-axis terms, including motor
locations, battery and payload. Ref6 wing loading is about 177.8 N/m^2 and
aspect ratio 2.32; neither establishes attitude response or transition trim.

## Conservative Validation Gates

1. Verify units, saved blade geometry, RPM limiting and whether 750 W is
   shaft or electrical. Audit any plugin overrides. Enforce separate
   torque/current, power and RPM limits; do not force commanded RPM through
   an overloaded prop. No controller tuning until this distinction is clear.
2. In a separate candidate installation, capture matched-condition static
   points at 8k, 9k, 10k, 11k, 12k, then approach 13k only if power allows.
   Record density, RPM, per-prop thrust, shaft torque/power, electrical
   power if available, actuator command and actual aircraft mass. Check
   P=Q*Omega and force balance; eliminate ground effect/interference or
   document it. Compare curves, not just hover command percentage.
3. Repeat at representative axial inflows and installed hover conditions.
   Establish achievable RPM versus airspeed at 750 W. A static match alone
   does not validate transition or forward flight.
4. Around hover, collect small independent command up/down steps on each
   motor. Log command, actual throttle, RPM and thrust at preferably 100 Hz
   or faster with simulation timestamps. Estimate dead time, 10-90% rise,
   settling and asymmetry. Repeat near higher load and during reduced bus
   voltage if modeled. Treat 0.3 s loaded thrust rise as a provisional
   investigation target, not a measured capability or universal stability
   criterion. Assess controller bandwidth and phase margin against the
   measured response, including delay.
5. Only then test restrained/staged 6 kg hover and transition, checking
   collective and differential authority under the measured worst loading.
   Nominal static thrust/weight near 1.90 at 13k is an analytical estimate,
   not established installed margin. A 20% thrust derating makes it about
   1.52 before further installation losses; no motor-out capability is claimed.

Bottom line: 2.60x is a defensible, conservatively rounded power-led chord
candidate. It is not uniquely optimal, an APC replica, X-Plane calibrated,
or a guarantee of the next flight. Fit simulator behavior to measured
thrust/power/loaded response in separate steps and retain uncertainty until
those tests exist.
