#!/usr/bin/env python3
"""Static annular BEM screening, not X-Plane calibration. Standard library only.

Run: python3 -B PROPULSION_REVIEW.py --scale 2.6
The default polar is the reviewed repository input. No files are written.
"""
import argparse
import hashlib
import json
from pathlib import Path
from math import *
from bisect import bisect_right

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--polar", type=Path, default=Path(__file__).resolve().parent/"aircraft/QuadTailsitterRef6/Airfoil/NACA 2412 (popular).afl")
parser.add_argument("--scale", type=float, default=2.6)
parser.add_argument("--annuli", type=int, default=160)
args = parser.parse_args()
if not 1 <= args.scale <= 5 or args.annuli < 20:
    parser.error("scale must be 1..5; annuli must be >=20")
raw = args.polar.read_bytes()
ps = []
headers = []
parts = raw.decode().split("alpha cl cd cm:")
for i, block in enumerate(parts[1:]):
    headers.append(parts[i].splitlines()[-1])
    rows = []
    for line in block.splitlines():
        fields = line.split()
        if len(fields) == 4:
            try:
                rows.append([float(v) for v in fields])
            except ValueError:
                pass
    if len(rows) < 100 or any(rows[j][0] >= rows[j+1][0] for j in range(len(rows)-1)):
        raise ValueError("Invalid polar block")
    ps.append(rows)
if not ps:
    raise ValueError("No polar blocks")
p = ps[0]
aa = [z[0] for z in p]
def polar(a):
 i=max(0,min(len(p)-2,bisect_right(aa,a)-1));f=(a-p[i][0])/(p[i+1][0]-p[i][0]);return [p[i][j]+f*(p[i+1][j]-p[i][j]) for j in (1,2)]
def bem(k,rpm=12000,hub=.2,loss=True,cls=1,cds=1,N=160,which=-1):
 rho=1.225;R=.127;B=2;om=rpm*pi/30;dr=R*(1-hub)/N;T=P=err=0;sections=[];multi=0
 for n in range(N):
  r=R*hub+(n+.5)*dr;c=k*(.039166667+(.019166667-.039166667)*r/R)*.3048;beta=atan(.2032/(2*pi*r))
  def eq(phi,ret=False):
   a=degrees(beta-phi);cl,cd=polar(a);cl*=cls;cd*=cds
   cn=cl*cos(phi)-cd*sin(phi);ct=cl*sin(phi)+cd*cos(phi)
   K=ct/cn if cn>0 else 1e6
   vi=om*r*tan(phi)/(1+K*tan(phi));u=vi*K;vt=om*r-u
   F=2/pi*acos(exp(-B*(R-r)/(2*r*max(abs(sin(phi)),.001)))) if loss else 1
   q=.5*rho*(vt*vt+vi*vi)*B*c;t=q*cn;qt=q*ct*r
   return (t,qt,a,c*hypot(vt,vi)/1.46e-5) if ret else t-4*pi*rho*r*vi*vi*F
  roots=[];last=.00001;fl=eq(last)
  for j in range(1,251):
   now=(beta+radians(2.4))*j/250;fn=eq(now)
   if fl*fn<0:
    lo=last;hi=now
    for z in range(35):
     mid=(lo+hi)/2
     if eq(lo)*eq(mid)<=0:hi=mid
     else:lo=mid
    roots.append((lo+hi)/2)
   last=now;fl=fn
  if not roots:raise Exception(('no root',r/R))
  if len(roots)>1:multi+=1
  phi=roots[which];err=max(err,abs(eq(phi)));t,q,a,re=eq(phi,True);T+=t*dr;P+=q*om*dr
  if n in [0,19,59,99,139,159]:sections.append((r/R,a,re))
 return T,P,err,multi,sections
def fit(target,index=0):
 lo=1;hi=5
 for i in range(20):
  mid=(lo+hi)/2
  if bem(mid)[index]<target:lo=mid
  else:hi=mid
 return (lo+hi)/2

def result(scale, rpm=12000, **kwargs):
    t, power, residual, multiple, sections = bem(scale, rpm=rpm, **kwargs)
    return dict(thrust_n=t, shaft_w=power, torque_nm=power/(rpm*pi/30),
                q_over_t_m=power/(rpm*pi/30)/t,
                max_thrust_residual_n_per_m=residual,
                annuli_with_multiple_roots=multiple,
                sampled_sections_r_over_R_alpha_deg_Re=sections)

k=args.scale
out = dict(
    status="ANALYTICAL SCREEN ONLY; NOT X-PLANE CALIBRATED",
    polar_path=str(args.polar), polar_sha256=hashlib.sha256(raw).hexdigest(),
    polar_headers=headers, primary_polar_block=0,
    assumptions=dict(rho_kg_m3=1.225, radius_m=.127, blades=2, pitch_m=.2032,
                     hub_fraction=.2, prandtl_tip_loss=True,
                     separate_hub_loss=False, compressibility=False,
                     reynolds_interpolation=False, root_selection="largest phi"),
    scale=k, root_chord_ft=k*.039166667, tip_chord_ft=k*.019166667,
    at_12000rpm=result(k,N=args.annuli),
    at_13000rpm=result(k,rpm=13000,N=args.annuli),
    alternative_smallest_phi=result(k,N=args.annuli,which=0),
    grid_checks={str(n):result(k,N=n) for n in (80,160,320)},
    sensitivity_low_lift_high_drag=result(k,cls=.85,cds=1.5,N=args.annuli),
    sensitivity_high_lift_low_drag=result(k,cls=1.15,cds=.8,N=args.annuli),
    hub_sensitivities={str(h):result(k,hub=h,N=args.annuli) for h in (.15,.25)},
    no_tip_loss=result(k,loss=False,N=args.annuli),
    comparison_2p5=result(2.5,N=args.annuli),
    power_match_scale=fit(577.465,1), thrust_match_scale=fit(26.271,0))
t=out["at_12000rpm"]["thrust_n"]
power=out["at_12000rpm"]["shaft_w"]
hover_rpm=12000*sqrt((6*9.80665/4)/t)
hover_power=power*(hover_rpm/12000)**3
out["uncanted_6kg_hover_constant_coefficient_scaling"]=dict(
    thrust_each_n=6*9.80665/4, rpm=hover_rpm, shaft_each_w=hover_power,
    q_over_t_m=hover_power/(hover_rpm*pi/30)/(6*9.80665/4),
    warning="Fixed polar and coefficients; changing Reynolds and installation omitted")
out["whole_polar_block_sensitivities"]={}
for i, block in enumerate(ps):
    p=block
    aa=[z[0] for z in p]
    out["whole_polar_block_sensitivities"][str(i)]=result(k,N=args.annuli)
print(json.dumps(out, indent=2, allow_nan=False))
