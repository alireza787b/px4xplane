#!/usr/bin/env python3
"""Reproducible hover/GoTo/Land review from a Ref6 ZIP (numpy, pyulog).

No extraction, flight control or simulator writes. Phase windows are derived
from recorded commands/mode changes, not wall-clock filenames.
"""
import argparse
import csv
import hashlib
import html
import io
import json
from pathlib import Path
import zipfile

import numpy as np
from pyulog import ULog


def stats(values):
    values=np.asarray(values,dtype=float)
    values=values[np.isfinite(values)]
    if not len(values):return {'samples':0}
    return dict(samples=len(values),min=float(values.min()),median=float(np.median(values)),
                max=float(values.max()),rms=float(np.sqrt(np.mean(values**2))))


def clock(data):
    return data['timestamp'].astype(float)/1e6


def interp(data,key,times):
    return np.interp(times,clock(data),data[key])


def plot(path, title, panels, lo, hi):
    lines=[f'<svg xmlns="http://www.w3.org/2000/svg" width="1050" height="{70+190*len(panels)}">',
           '<rect width="100%" height="100%" fill="white"/>',
           f'<text x="75" y="25" font-family="sans-serif">{html.escape(title)}</text>']
    for index,(label,series) in enumerate(panels):
        top=65+190*index
        clipped=[v[(t>=lo)&(t<=hi)&np.isfinite(v)] for _,t,v in series]
        all_values=np.concatenate(clipped)
        low,high=float(all_values.min()),float(all_values.max())
        span=max(high-low,.01)
        lines.append(f'<text x="75" y="{top-12}" font-family="sans-serif">{html.escape(label)} ({low:.2f} .. {high:.2f})</text>')
        for (name,times,values),color in zip(series,('#176c9c','#c34a26','#26844b')):
            ids=np.flatnonzero((times>=lo)&(times<=hi)&np.isfinite(values))
            # Binned extrema retain short rate spikes without huge SVGs.
            points=[]
            for group in np.array_split(ids,min(800,len(ids))):
                if not len(group):continue
                for i in sorted(set((group[np.argmin(values[group])],group[np.argmax(values[group])]))):
                    points.append(f'{75+(times[i]-lo)/(hi-lo)*920:.1f},{top+135-(values[i]-low)/span*135:.1f}')
            lines.append(f'<polyline points="{" ".join(points)}" fill="none" stroke="{color}" stroke-width="1"/>')
        for sec in np.linspace(lo,hi,8):
            lines.append(f'<text x="{75+(sec-lo)/(hi-lo)*920:.1f}" y="{top+153}" font-family="sans-serif" font-size="11">{sec:.0f}s</text>')
    lines.append('</svg>');path.write_text('\n'.join(lines))


def analyze(archive,output):
    output.mkdir(parents=True,exist_ok=False)
    with zipfile.ZipFile(archive) as zipped:
        def read(suffix):
            matches=[n for n in zipped.namelist() if n.endswith(suffix)]
            if len(matches)!=1:raise ValueError('Ambiguous/missing '+suffix)
            return zipped.read(matches[0])
        identity=json.loads(read('/identity.json'))
        u=ULog(io.BytesIO(read('.ulg')))
        def data(name):return next(x.data for x in u.data_list if x.name==name and x.multi_id==0)
        command=data('vehicle_command');status=data('vehicle_status');landed=data('vehicle_land_detected')
        def first_command(number):return float(clock(command)[command['command']==number][0])
        takeoff=first_command(22);goto=first_command(192)
        land=float(clock(status)[status['nav_state']==18][0])
        touchdown=float(clock(landed)[(clock(landed)>land)&(landed['landed']==1)][0])
        phases={'hover':(max(takeoff+10,goto-11),goto-1),'goto':(goto+1,min(goto+35,land-2)),
                'land':(land+2,touchdown-2)}
        lp=data('vehicle_local_position');lps=data('vehicle_local_position_setpoint')
        angular=data('vehicle_angular_velocity');rates=data('vehicle_rates_setpoint')
        att=data('vehicle_attitude');asp=data('vehicle_attitude_setpoint');motors=data('actuator_motors')
        hte=data('hover_thrust_estimate')
        params=dict(u.initial_parameters)
        for _,key,value in u.changed_parameters:params[key]=value
        metrics={'archive_sha256':hashlib.sha256(archive.read_bytes()).hexdigest(),'identity':identity,
                 'events_boot_s':{'takeoff_request':takeoff,'goto':goto,'land':land,'touchdown':touchdown},
                 'vtol_states':np.unique(data('vtol_vehicle_status')['vehicle_vtol_state']).tolist(),
                 'messages':[{'t_s':m.timestamp/1e6,'text':m.message} for m in u.logged_messages],
                 'effective_parameters':params,'phases':{}}
        for label,(lo,hi) in phases.items():
            phase={'boot_window_s':[lo,hi]};pt=clock(lp);mask=(pt>=lo)&(pt<=hi);pt=pt[mask]
            phase['position_error_m']={k:stats(lp[k][mask]-interp(lps,k,pt)) for k in ('x','y','z')}
            phase['velocity_error_mps']={k:stats(lp[k][mask]-interp(lps,k,pt)) for k in ('vx','vy','vz')}
            rt=clock(angular);mask=(rt>=lo)&(rt<=hi);rt=rt[mask]
            phase['rate_error_rad_s']={k:stats(angular[f'xyz[{i}]'][mask]-interp(rates,k,rt)) for i,k in enumerate(('roll','pitch','yaw'))}
            at=clock(att);mask=(at>=lo)&(at<=hi);at=at[mask]
            qa=np.array([att[f'q[{i}]'][mask] for i in range(4)]).T
            qs=np.array([interp(asp,f'q_d[{i}]',at) for i in range(4)]).T
            qa/=np.linalg.norm(qa,axis=1)[:,None];qs/=np.linalg.norm(qs,axis=1)[:,None]
            phase['attitude_error_deg']=stats(np.degrees(2*np.arccos(np.clip(abs(np.sum(qa*qs,axis=1)),0,1))))
            mask=(clock(motors)>=lo)&(clock(motors)<=hi)
            phase['motor_command']=[stats(motors[f'control[{i}]'][mask]) for i in range(4)]
            mask=(clock(hte)>=lo)&(clock(hte)<=hi);phase['hover_thrust_estimate']=stats(hte['hover_thrust'][mask])
            metrics['phases'][label]=phase
        rows=csv.DictReader(io.StringIO(read('/truth/frames.csv').decode()))
        truth={key:[] for key in ('time','alt','vy','mass','payload','wind')}
        pitch_moment=[]
        acf={parts[1]:parts[2] for line in read('/inputs/QuadTailsitterRef6.acf').decode().splitlines()
             if len(parts:=line.split(maxsplit=2))==3 and parts[0]=='P'}
        pitch_arms=np.array([-float(acf[f'_engn/{i}/_part_z'])*.3048 for i in range(4)])
        fields=('sim/time/total_flight_time_sec','sim/flightmodel/position/elevation',
                'sim/flightmodel/position/local_vy','sim/flightmodel/weight/m_total',
                'sim/flightmodel/weight/m_fixed','sim/weather/aircraft/wind_now_speed_msc')
        for row in rows:
            for key,field in zip(truth,fields):truth[key].append(float(row[field]))
            thrust=np.array([float(v) for v in row['sim/flightmodel/engine/POINT_thrust'].split(';')[:4]])
            pitch_moment.append(float(pitch_arms@thrust))
        truth={k:np.asarray(v) for k,v in truth.items()}
        metrics['truth']={k:stats(truth[k]) for k in ('mass','payload','wind')}
        gt=data('vehicle_global_position_groundtruth');gt_t=clock(gt)
        mask=(gt_t>=takeoff)&(gt_t<=touchdown)
        guess=truth['time'][0]-gt_t[0]
        errors=[]
        for offset in np.arange(guess-10,guess+10,.005):
            if gt_t[mask].min()+offset<truth['time'][0] or gt_t[mask].max()+offset>truth['time'][-1]:continue
            residual=np.interp(gt_t[mask]+offset,truth['time'],truth['alt'])-gt['alt'][mask]
            errors.append((float(np.mean(residual**2)),float(offset)))
        if not errors:raise ValueError('No non-extrapolated truth clock fit')
        mse,offset=min(errors)
        mask=(clock(lp)>takeoff)&(clock(lp)<touchdown)
        metrics['truth']['clock_fit']={'sim_minus_boot_s':offset,'groundtruth_altitude_rmse_m':float(np.sqrt(mse)),
            'independent_estimator_vz_error_mps':stats(lp['vz'][mask]+np.interp(clock(lp)[mask]+offset,truth['time'],truth['vy']))}
        metrics['estimator_accel_bias_mps2']=[stats(data('estimator_sensor_bias')[f'accel_bias[{i}]']) for i in range(3)]
        lo,hi=phases['goto'];times=np.arange(lo,hi,.02)
        actual=interp(angular,'xyz[1]',times)
        fits=[]
        for delay in np.arange(0,3,.02):
            desired=interp(rates,'pitch',times-delay)
            fits.append((float(np.corrcoef(desired,actual)[0,1]),float(delay)))
        corr,delay=max(fits)
        metrics['pitch_rate_crosscorrelation']={'lag_s':delay,'correlation':corr,
            'warning':'Closed-loop time-domain lag, not a pure actuator transport delay or open-loop plant identification'}
        torque=data('vehicle_torque_setpoint');physical=np.interp(times+offset,truth['time'],pitch_moment)
        fits=[]
        for delay in np.arange(0,1,.025):
            desired=interp(torque,'xyz[1]',times-delay)
            x=np.c_[desired,np.ones(len(desired))]
            coefficients=np.linalg.lstsq(x,physical,rcond=None)[0]
            fits.append((float(np.mean((physical-x@coefficients)**2)),float(delay),coefficients.tolist(),
                         float(np.corrcoef(desired,physical)[0,1])))
        mse,delay,coeff,corr=min(fits)
        metrics['axial_pitch_moment_response']={'fit_lag_s':delay,'nm_per_normalized_torque':coeff[0],
            'correlation':corr,'fit_rms_nm':float(np.sqrt(mse)),
            'warning':'Axial prop-thrust moment proxy only; omits lateral cant forces, aerodynamic and gyroscopic moments. Does not identify rigid-body inertia.'}
        goto_i=np.flatnonzero(command['command']==192)[0]
        radius=6371000.
        target=np.array([(command['param5'][goto_i]-lp['ref_lat'][-1])*np.pi/180*radius,
                         (command['param6'][goto_i]-lp['ref_lon'][-1])*np.pi/180*radius*np.cos(lp['ref_lat'][-1]*np.pi/180)])
        start=np.array([interp(lp,k,goto) for k in ('x','y')]);direction=target-start
        length=float(np.linalg.norm(direction));unit=direction/length
        mask=(clock(lp)>goto)&(clock(lp)<land)
        xy=np.c_[lp['x'][mask],lp['y'][mask]]-start
        metrics['goto_geometry']={'start_xy_m':start.tolist(),'target_xy_m':target.tolist(),'distance_m':length,
            'cross_track_m':stats(xy@np.array([-unit[1],unit[0]])),
            'along_track_beyond_target_max_m':float(np.max(xy@unit)-length),
            'distance_to_target_before_land_m':float(np.linalg.norm(xy[-1]-direction))}
        metrics['warning']='No FW or back-transition data; MC performance does not validate either mode.'
        (output/'metrics.json').write_text(json.dumps(metrics,indent=2,allow_nan=False)+'\n')
        panels=[('MC '+axis+' rate rad/s (blue actual, red command)',[
                 ('actual',clock(angular),angular[f'xyz[{i}]']),('command',clock(rates),rates[axis])]) for i,axis in enumerate(('roll','pitch','yaw'))]
        plot(output/'rates.svg','R6 MC GoTo rate tracking; PX4 boot seconds',panels,goto-4,land)
        panels=[(axis+' position m (blue actual, red smoothed setpoint)',[
                 ('actual',clock(lp),lp[axis]),('setpoint',clock(lps),lps[axis])]) for axis in ('x','y','z')]
        plot(output/'tracking.svg','R6 MC position tracking; not a FW flight',panels,takeoff,touchdown)
        return metrics


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('archive',type=Path);parser.add_argument('output',type=Path)
    args=parser.parse_args();result=analyze(args.archive,args.output)
    print(json.dumps({k:result[k] for k in ('events_boot_s','truth','pitch_rate_crosscorrelation')},indent=2))
