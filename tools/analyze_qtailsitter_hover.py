#!/usr/bin/env python3
"""Stream a Ref6 ZIP for a bounded hover review; never extract flight artifacts.

Requires numpy and pyulog. Writes only metrics.json and hover.svg into a NEW
output directory. Closed-loop response is not an open-loop identification.
"""
import argparse
import csv
import hashlib
import io
import json
from pathlib import Path
import zipfile

import numpy as np
from pyulog import ULog
from analyze_qtailsitter_design import CaptureFrames


def analyze(archive, output):
    output.mkdir(parents=True, exist_ok=False)
    with zipfile.ZipFile(archive) as zipped:
        def member(suffix):
            matches = [n for n in zipped.namelist() if n.endswith(suffix)]
            if len(matches) != 1:
                raise ValueError('Ambiguous/missing '+suffix)
            return matches[0]
        ulog = ULog(io.BytesIO(zipped.read(member('.ulg'))))
        def topic(name):
            return next(d.data for d in ulog.data_list if d.name == name and d.multi_id == 0)
        def clock(data):
            return data['timestamp'].astype(float)/1e6
        position = topic('vehicle_local_position')
        angular = topic('vehicle_angular_velocity')
        rates = topic('vehicle_rates_setpoint')
        commands = topic('vehicle_command')
        hover = topic('hover_thrust_estimate')
        masks = commands['command'] == 22
        takeoff = float(clock(commands)[masks][0])
        climb = (clock(position) > takeoff) & (position['z'] < -.5)
        first_climb = float(clock(position)[climb][0]) if np.any(climb) else None
        steady = (clock(hover) >= 65) & (clock(hover) <= 85)
        hte = float(np.median(hover['hover_thrust'][steady]))
        mass, payload = [], []
        with CaptureFrames(archive) as stream:
            for row in csv.DictReader(stream):
                mass.append(float(row['sim/flightmodel/weight/m_total']))
                payload.append(float(row['sim/flightmodel/weight/m_fixed']))
        metrics = {
            'archive_sha256': hashlib.sha256(archive.read_bytes()).hexdigest(),
            'identity': json.loads(zipped.read(member('identity.json'))),
            'actual_mass_median_kg': float(np.median(mass)),
            'fixed_payload_median_kg': float(np.median(payload)),
            'takeoff_request_boot_s': takeoff,
            'first_estimated_climb_above_half_meter_boot_s': first_climb,
            'delay_request_to_half_meter_s': first_climb-takeoff if first_climb else None,
            'hte_median_boot_65_85': hte,
            'mass_corrected_hover_seed_2kg': hte*(2/np.median(mass))**1.5,
            'vtol_state_values': np.unique(topic('vtol_vehicle_status')['vehicle_vtol_state']).tolist(),
            'warning': 'Closed-loop flight, not independently excited plant identification. Truth and PX4 clocks are not assumed identical.'}
        (output/'metrics.json').write_text(json.dumps(metrics, indent=2)+'\n')
        series = [('Altitude above EKF origin (m)', clock(position), -position['z'], None, None),
                  ('MC pitch rate (rad/s)', clock(angular), angular['xyz[1]'], clock(rates), rates['pitch']),
                  ('MC roll rate (rad/s)', clock(angular), angular['xyz[0]'], clock(rates), rates['roll'])]
        svg = ['<svg xmlns="http://www.w3.org/2000/svg" width="1000" height="630" viewBox="0 0 1000 630">',
               '<rect width="1000" height="630" fill="white"/>',
               '<text x="70" y="22" font-family="sans-serif">R5 hover failure: PX4 boot seconds; blue actual, red rate setpoint</text>']
        for panel,(label,t,y,st,sy) in enumerate(series):
            keep=(t>=35)&(t<=100)
            low,high=float(np.min(y[keep])),float(np.max(y[keep]))
            if st is not None:
                sk=(st>=35)&(st<=100)
                low=min(low,float(np.min(sy[sk])));high=max(high,float(np.max(sy[sk])))
            span=max(high-low,.01); top=65+190*panel
            svg.append(f'<text x="70" y="{top-12}" font-family="sans-serif">{label}: {low:.2f} to {high:.2f}</text>')
            for times,values,color in ((t,y,'#176c9c'),(st,sy,'#bd342d')):
                if times is None:continue
                indices=np.flatnonzero((times>=35)&(times<=100))
                indices=indices[::max(1,len(indices)//1500)]
                points=' '.join(f'{70+(float(times[i])-35)/65*880:.1f},{top+140-(float(values[i])-low)/span*140:.1f}' for i in indices)
                svg.append(f'<polyline points="{points}" fill="none" stroke="{color}" stroke-width="1"/>')
            for sec in range(40,101,10):
                x=70+(sec-35)/65*880
                svg.append(f'<text x="{x}" y="{top+159}" font-family="sans-serif" font-size="11">{sec}</text>')
        svg.append('</svg>')
        (output/'hover.svg').write_text('\n'.join(svg))
        print(json.dumps(metrics,indent=2))


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('archive',type=Path)
    parser.add_argument('output',type=Path)
    args=parser.parse_args()
    analyze(args.archive,args.output)
