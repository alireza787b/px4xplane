#!/usr/bin/env python3
"""Read a Ref6 flight ZIP without extraction; review FW/back/MC handoff.

Requires numpy and pyulog. Frame conversion follows the pinned PX4 tailsitter
contract: q_FW = q_MC * Ry(+90 degrees), omega_FW = (-r_MC, q_MC, p_MC).
Closed-loop correlations and local regressions are descriptive, not open-loop
identification, stability margins or proof of causation.
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

from analyze_qtailsitter_tracking import clock, interp, plot, stats
from analyze_qtailsitter_design import CaptureFrames


def multiply(a, b):
    a, b = np.broadcast_arrays(np.asarray(a), np.asarray(b))
    w, x, y, z = a.T
    v, i, j, k = b.T
    return np.c_[w*v-x*i-y*j-z*k, w*i+x*v+y*k-z*j,
                 w*j-x*k+y*v+z*i, w*k+x*j-y*i+z*v]


def euler(q):
    q = np.asarray(q)
    q = q / np.linalg.norm(q, axis=1)[:, None]
    w, x, y, z = q.T
    return np.degrees(np.c_[np.arctan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
                            np.arcsin(np.clip(2*(w*y-z*x), -1, 1)),
                            np.arctan2(2*(w*z+x*y), 1-2*(y*y+z*z))])


def quaternion(data, prefix):
    return np.c_[[data[f'{prefix}[{i}]'] for i in range(4)]].T


def peak(times, values, lo, hi):
    grid = np.arange(lo, hi, .02)
    v = np.interp(grid, times, values)
    spectrum = abs(np.fft.rfft(v-v.mean()))**2
    freq = np.fft.rfftfreq(len(v), .02)
    valid = np.flatnonzero((freq >= .06) & (freq <= 10))
    return float(freq[valid[np.argmax(spectrum[valid])]])


def lag_fit(times, actual, desired):
    candidates = []
    for delay in np.arange(0, 2.01, .02):
        command = np.interp(times-delay, times, desired)
        if np.std(command) < 1e-8 or np.std(actual) < 1e-8:
            continue
        candidates.append((float(np.corrcoef(actual, command)[0, 1]), float(delay)))
    if not candidates:
        return None
    correlation, lag = max(candidates)
    return {'lag_s': lag, 'correlation': correlation,
            'caveat': 'Closed-loop periodic correlation, not actuator delay or a causal plant fit.'}


def mean_or_none(values):
    values = np.asarray(values)
    values = values[np.isfinite(values)]
    return float(np.mean(values)) if len(values) else None


def local_roll_screens(angular, rates, torque, integral, lo, hi, dt=.02):
    """Screen rate-P candidates, without pretending to simulate position/landing."""
    times = np.arange(lo-1, hi+1, dt)
    rate = interp(angular, 'xyz[0]', times)
    smooth = np.convolve(rate, np.ones(11)/11, mode='same')
    acceleration = np.gradient(smooth, dt)
    use = (times >= lo) & (times < hi)
    fits = []
    for delay in np.arange(0, .31, dt):
        command = interp(torque, 'xyz[0]', times-delay)
        x = np.c_[command[use], smooth[use], np.ones(np.count_nonzero(use))]
        coefficients = np.linalg.lstsq(x, acceleration[use], rcond=None)[0]
        residual = acceleration[use]-x@coefficients
        r2 = 1-float(np.mean(residual**2)/max(np.var(acceleration[use]), 1e-12))
        fits.append((r2, float(delay), coefficients))
    r2, delay, (a, b, offset) = max(fits, key=lambda fit: fit[0])
    simulations = {}
    step = .002
    grid = np.arange(lo, hi, step)
    desired = interp(rates, 'roll', grid)
    for p in (.031815, .04, .06, .08):
        w = float(interp(angular, 'xyz[0]', np.array([lo]))[0])
        integ = float(interp(integral, 'rollspeed_integ', np.array([lo]))[0])
        accel = 0.; commands = []; simulated = []
        delay_steps = int(round(delay/step))
        for i, target in enumerate(desired):
            error = target-w
            integ += .007954*error*step
            command = p*error+integ-.000636*accel
            commands.append(command)
            applied = commands[max(0, i-delay_steps)]
            accel = a*applied+b*w+offset
            w += step*accel
            simulated.append(w)
        simulations[str(p)] = {'rate_error_rms_rad_s': float(np.sqrt(np.mean((np.array(simulated)-desired)**2))),
                               'max_abs_normalized_torque': float(np.max(np.abs(commands)))}
    return {'boot_window_s': [lo, hi], 'a_torque': float(a), 'b_rate': float(b),
            'offset': float(offset), 'fit_delay_s': delay, 'smoothed_acceleration_r2': r2,
            'candidate_P_screens': simulations,
            'caveat': 'In-sample CLOSED-LOOP local model and fixed recorded rate demand. '
                      '0.22s smoothing changes apparent delay; no actuator saturation, '
                      'position feedback, braking, aerodynamic regime change or stability guarantee.'}


def analyze(archive, output):
    output.mkdir(parents=True, exist_ok=False)
    with zipfile.ZipFile(archive) as z:
        def read(suffix):
            names = [n for n in z.namelist() if n.endswith(suffix)]
            if len(names) != 1:
                raise ValueError('Ambiguous/missing archive member: '+suffix)
            return z.read(names[0])
        u = ULog(io.BytesIO(read('.ulg')))
        def d(name, mid=0):
            return next(x.data for x in u.data_list if x.name == name and x.multi_id == mid)
        identity = json.loads(read('/identity.json'))
        vt = d('vtol_vehicle_status')
        changes = np.r_[0, np.flatnonzero(np.diff(vt['vehicle_vtol_state']))+1]
        transitions = [{'boot_s': float(clock(vt)[i]), 'state': int(vt['vehicle_vtol_state'][i])}
                       for i in changes]
        def first_state(state, after=0):
            return next(x['boot_s'] for x in transitions if x['state'] == state and x['boot_s'] > after)
        front = first_state(1)
        fw = first_state(4, front)
        back = first_state(2, fw)
        mc = first_state(3, back)
        status = d('vehicle_status')
        land = float(clock(status)[(status['nav_state'] == 18) & (clock(status) > mc)][0])
        end = float(clock(status)[-1])
        windows = {'hover': (front-18, front-3), 'front': (front, fw),
                   'FW': (fw+2, back-1), 'back': (back, mc),
                   'MC_recovery': (mc+2, land-1), 'Land': (land+3, end-2)}
        att = d('vehicle_attitude'); asp = d('vehicle_attitude_setpoint')
        angular = d('vehicle_angular_velocity'); rates = d('vehicle_rates_setpoint')
        lp = d('vehicle_local_position'); lps = d('vehicle_local_position_setpoint')
        motors = d('actuator_motors'); torque = d('vehicle_torque_setpoint')
        mc_angles = euler(quaternion(att, 'q'))
        fw_angles = euler(multiply(quaternion(att, 'q'), [2**-.5, 0, 2**-.5, 0]))
        sp_angles = euler(quaternion(asp, 'q_d'))
        params = dict(u.initial_parameters)
        for _, key, value in u.changed_parameters:
            params[key] = value
        metrics = {'archive_sha256': hashlib.sha256(archive.read_bytes()).hexdigest(),
                   'identity': identity, 'vtol_events': transitions, 'effective_parameters': params,
                   'events_boot_s': dict(front=front, FW=fw, back=back, MC=mc, Land=land, end=end),
                   'dropouts': len(u.dropouts), 'phases': {},
                   'messages': [{'boot_s': m.timestamp/1e6, 'text': m.message} for m in u.logged_messages]}
        metrics['local_roll_screens'] = [local_roll_screens(angular, rates, torque, d('rate_ctrl_status'), lo, lo+25)
            for lo in np.arange(land+20, end-40, 35)]
        for label, (lo, hi) in windows.items():
            times = np.arange(lo, hi, .02)
            phase = {'boot_window_s': [lo, hi]}
            phase['rate_tracking'] = {}
            for i, axis in enumerate(('roll', 'pitch', 'yaw')):
                actual = interp(angular, f'xyz[{i}]', times)
                desired = interp(rates, axis, times)
                phase['rate_tracking'][axis] = {'actual_rad_s': stats(actual),
                    'error_rad_s': stats(actual-desired),
                    'dominant_hz': peak(times, actual, lo, hi),
                    'lag_fit': lag_fit(times, actual, desired)}
            mask = (clock(att) >= lo) & (clock(att) <= hi)
            angles = fw_angles if label == 'FW' else mc_angles
            phase['euler_deg'] = {axis: stats(angles[mask, i]) for i, axis in enumerate(('roll', 'pitch', 'yaw'))}
            phase['euler_frame'] = 'FW' if label == 'FW' else 'MC; roll/yaw singular near vertical pitch in front/back'
            phase['velocity_down_mps'] = stats(interp(lp, 'vz', times))
            phase['command_down_mps'] = stats(interp(lps, 'vz', times))
            phase['horizontal_speed_mps'] = stats(np.hypot(interp(lp, 'vx', times), interp(lp, 'vy', times)))
            phase['position_error_xy_m'] = stats(np.hypot(interp(lp, 'x', times)-interp(lps, 'x', times),
                                                         interp(lp, 'y', times)-interp(lps, 'y', times)))
            phase['altitude_above_local_origin_m'] = stats(-interp(lp, 'z', times))
            phase['motor_commands'] = [stats(interp(motors, f'control[{i}]', times)) for i in range(4)]
            phase['fraction_any_motor_ge_099'] = float(np.mean(np.max([interp(motors, f'control[{i}]', times) for i in range(4)], axis=0) >= .99))
            phase['fraction_any_motor_le_001'] = float(np.mean(np.min([interp(motors, f'control[{i}]', times) for i in range(4)], axis=0) <= .01))
            for mid in (0, 1):
                rate = d('rate_ctrl_status', mid)
                phase[f'rate_integrals_{mid}'] = {axis: stats(interp(rate, axis+'speed_integ', times)) for axis in ('roll', 'pitch', 'yaw')}
            for topic in (x for x in u.data_list if x.name == 'debug_array'):
                dd = topic.data
                if not len(dd['id']):
                    continue
                name = 'tail_diag' if dd['id'][0] == 102 else 'auto_diag'
                mask = (clock(dd) >= lo) & (clock(dd) <= hi)
                phase[name] = {key: stats(dd[key][mask]) for key in dd if key.startswith('data[')}
            metrics['phases'][label] = phase

        # Groundtruth topic comparison catches attitude/rate disagreement without a
        # truth clock fit; the independent simulator CSV checks velocity/actuation.
        gt_att = d('vehicle_attitude_groundtruth'); gt_ang = d('vehicle_angular_velocity_groundtruth')
        times = np.arange(front-18, end-2, .02)
        qa = np.array([interp(att, f'q[{i}]', times) for i in range(4)]).T
        qg = np.array([interp(gt_att, f'q[{i}]', times) for i in range(4)]).T
        qa /= np.linalg.norm(qa, axis=1)[:, None]; qg /= np.linalg.norm(qg, axis=1)[:, None]
        metrics['estimator_truth'] = {'quaternion_error_deg': stats(np.degrees(2*np.arccos(np.clip(abs(np.sum(qa*qg, axis=1)), 0, 1)))),
            'rate_error_rad_s': [stats(interp(angular, f'xyz[{i}]', times)-interp(gt_ang, f'xyz[{i}]', times)) for i in range(3)],
            'accel_bias_mps2': [stats(d('estimator_sensor_bias')[f'accel_bias[{i}]']) for i in range(3)]}
        truth = None
        if not any(n.endswith(('/truth/frames.csv', '/truth/frames.csv.gz')) for n in z.namelist()):
            metrics['truth'] = {'available': False, 'reason': 'Capture absent; no independent CSV velocity/actuator-force review possible.'}
        else:
            fields = {'time': 'sim/time/total_flight_time_sec', 'alt': 'sim/flightmodel/position/elevation',
                      'vy': 'sim/flightmodel/position/local_vy', 'mass': 'sim/flightmodel/weight/m_total',
                      'payload': 'sim/flightmodel/weight/m_fixed', 'wind': 'sim/weather/aircraft/wind_now_speed_msc'}
            vector_fields = {'thrust': 'sim/flightmodel/engine/POINT_thrust',
                             'throttle': 'sim/flightmodel/engine/ENGN_thro_use',
                             'power': 'sim/flightmodel/engine/ENGN_power'}
            truth = {key: [] for key in (*fields, *vector_fields)}
            with CaptureFrames(archive) as stream:
                for row in csv.DictReader(stream):
                    for key, field in fields.items():
                        truth[key].append(float(row[field]))
                    for key, field in vector_fields.items():
                        truth[key].append([float(v) for v in row[field].split(';')[:4]])
            truth = {key: np.asarray(value) for key, value in truth.items()}
            if np.any(np.diff(truth['time']) <= 0):
                raise ValueError('Non-monotonic truth clock; segment before fitting')
            gt = d('vehicle_global_position_groundtruth')
            gt_times = clock(gt)
            # A runner snapshot may finish before the last ULog sample. Fit a
            # covered interior window; never extrapolate either stream afterward.
            mask = (gt_times >= front-18) & (gt_times <= end-20)
            guess = truth['time'][0]-gt_times[0]
            fits = []
            for offset in np.arange(guess-10, guess+10, .005):
                if gt_times[mask].min()+offset < truth['time'][0] or gt_times[mask].max()+offset > truth['time'][-1]:
                    continue
                residual = np.interp(gt_times[mask]+offset, truth['time'], truth['alt'])-gt['alt'][mask]
                fits.append((float(np.mean(residual**2)), float(offset)))
            if not fits:
                raise ValueError('Truth clock could not be fitted without extrapolation')
            mse, offset = min(fits)
            if np.sqrt(mse) > .1:
                raise ValueError('Truth clock fit residual exceeds 0.1m; inspect clock coverage before interpreting CSV')
            metrics['truth'] = {key: stats(truth[key]) for key in ('mass', 'payload', 'wind')}
            metrics['truth']['clock_fit'] = {'sim_minus_boot_s': offset, 'altitude_rms_m': float(np.sqrt(mse))}
            times = times[(times+offset >= truth['time'][0]) & (times+offset <= truth['time'][-1])]
            metrics['truth']['estimator_vz_error_mps'] = stats(interp(lp, 'vz', times)+np.interp(times+offset, truth['time'], truth['vy']))
            metrics['truth']['motor_channel_error'] = [stats(interp(motors, f'control[{i}]', times)-np.interp(times+offset, truth['time'], truth['throttle'][:, i])) for i in range(4)]
            acf = {parts[1]: parts[2] for line in read('/inputs/QuadTailsitterRef6.acf').decode().splitlines()
                   if len(parts := line.split(maxsplit=2)) == 3 and parts[0] == 'P'}
            # Axial-force proxy only. Cant, aerodynamic and gyroscopic moments are
            # omitted deliberately; this is NOT the net rigid-body moment.
            levers = np.array([[-float(acf[f'_engn/{i}/_part_x'])*.3048 for i in range(4)],
                               [-float(acf[f'_engn/{i}/_part_z'])*.3048 for i in range(4)]])
            moments = truth['thrust'] @ levers.T
            for label in ('FW', 'back', 'Land'):
                lo, hi = windows[label]
                tt = np.arange(max(lo+.5, truth['time'][0]-offset), min(hi-.5, truth['time'][-1]-offset), .02)
                result = {}
                for axis in (0, 1):
                    physical = np.interp(tt+offset, truth['time'], moments[:, axis])
                    candidates = []
                    for delay in np.arange(0, .51, .02):
                        command = interp(torque, f'xyz[{axis}]', tt-delay)
                        coefficients = np.linalg.lstsq(np.c_[command, np.ones(len(tt))], physical, rcond=None)[0]
                        predicted = np.c_[command, np.ones(len(tt))] @ coefficients
                        candidates.append((float(np.mean((physical-predicted)**2)), float(delay), coefficients.tolist(),
                                           float(np.corrcoef(command, physical)[0, 1])))
                    error, delay, coefficients, correlation = min(candidates)
                    result[str(axis)] = {'nm_per_normalized_torque': coefficients[0], 'offset_nm': coefficients[1],
                                        'lag_s': delay, 'correlation': correlation, 'residual_rms_nm': float(np.sqrt(error))}
                metrics['phases'][label]['axial_force_moment_proxy'] = result
        metrics['caveats'] = ['Not a validated plant identification or stability guarantee.',
            'Moment proxy, when available, omits cant, aerodynamics and gyroscopic moments.',
            'GT topics share the plugin transform; topic agreement alone does not prove all sensor paths correct.',
            'A mode label is not proof of upright recovery; inspect the logged exit condition.']
        auto = next(x.data for x in u.data_list if x.name == 'debug_array' and x.data['id'][0] == 101)
        after_land = (clock(auto) >= land) & (clock(auto) <= end)
        active = after_land & (auto['data[1]'] > .5)
        inactive = after_land & ~active
        metrics['braking'] = {'activation_count_final': int(auto['data[28]'][-1]),
            'land_active_sample_fraction': float(np.mean(auto['data[1]'][after_land] > .5)),
            'activation_reasons': np.unique(auto['data[2]'][auto['data[28]'] > 0]).astype(int).tolist(),
            'raw_land_down_mps': stats(auto['data[12]'][after_land]),
            'active_mean_actual_down_mps': mean_or_none(auto['data[21]'][active]),
            'inactive_mean_actual_down_mps': mean_or_none(auto['data[21]'][inactive]),
            'active_output_minus_actual_down_mps': stats(auto['data[24]'][active]-auto['data[21]'][active])}
        # One row per boot second: keep commands, motor range and physical
        # response together rather than inferring recovery from a mode label.
        tail = next(x.data for x in u.data_list if x.name == 'debug_array' and x.data['id'][0] == 102)
        with (output/'back_seconds.csv').open('w', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(['boot_s','mc_pitch_deg','sp_pitch_deg','pitch_rate_rad_s','pitch_rate_sp_rad_s',
                             'pitch_torque','motor_min','motor_max','horizontal_speed_mps','down_mps',
                             'height_above_origin_m','back_exit_reason_latched'])
            for sec in np.arange(np.ceil(back), np.floor(mc+10)+1):
                writer.writerow([round(sec, 3), float(np.interp(sec, clock(att), mc_angles[:, 1])),
                    float(np.interp(sec, clock(asp), sp_angles[:, 1])),
                    float(interp(angular, 'xyz[1]', sec)), float(interp(rates, 'pitch', sec)),
                    float(interp(torque, 'xyz[1]', sec)),
                    min(float(interp(motors, f'control[{i}]', sec)) for i in range(4)),
                    max(float(interp(motors, f'control[{i}]', sec)) for i in range(4)),
                    float(np.hypot(interp(lp, 'vx', sec), interp(lp, 'vy', sec))),
                    float(interp(lp, 'vz', sec)), -float(interp(lp, 'z', sec)),
                    int(interp(tail, 'data[1]', sec))])
        # FW attitude setpoint is already in FW coordinates; rate setpoint is MC.
        plot(output/'fw.svg', 'R8 FW: correct aircraft frame, actual blue / command red', [
            ('FW bank deg', [('actual', clock(att), fw_angles[:, 0]), ('command', clock(asp), sp_angles[:, 0])]),
            ('FW roll rate rad/s', [('actual', clock(angular), -angular['xyz[2]']), ('command', clock(rates), -rates['yaw'])]),
            ('MC yaw motor torque', [('torque', clock(torque), torque['xyz[2]'])])], fw+1, back-.2)
        plot(output/'recovery.svg', 'R8 MC recovery and Land: actual blue / command red', [
            ('MC roll deg (back-transition Euler singularity omitted)', [
                ('actual', clock(att), np.where(clock(att) >= mc, mc_angles[:, 0], np.nan)),
                ('command', clock(asp), np.where(clock(asp) >= mc, sp_angles[:, 0], np.nan))]),
            ('MC roll rate rad/s', [('actual', clock(angular), angular['xyz[0]']), ('command', clock(rates), rates['roll'])]),
            ('MC pitch deg', [('actual', clock(att), mc_angles[:, 1]), ('command', clock(asp), sp_angles[:, 1])]),
            ('Altitude m above local origin', [('actual', clock(lp), -lp['z'])]),
            ('Down velocity m/s', [('actual', clock(lp), lp['vz']), ('command', clock(lps), lps['vz'])])], back, min(end-2, land+45))
        plot(output/'land.svg', 'R8 sustained Land oscillation, actual blue / command red', [
            ('MC roll deg', [('actual', clock(att), mc_angles[:, 0]), ('command', clock(asp), sp_angles[:, 0])]),
            ('MC roll rate rad/s', [('actual', clock(angular), angular['xyz[0]']), ('command', clock(rates), rates['roll'])]),
            ('North velocity m/s', [('actual', clock(lp), lp['vx']), ('command', clock(lps), lps['vx'])]),
            ('Down velocity m/s', [('actual', clock(lp), lp['vz']), ('command', clock(lps), lps['vz'])])], end-52, end-2)
        plot(output/'braking.svg', 'R8 Land accepted: repeated all-axis braking interrupts descent', [
            ('Braking active 0/1', [('active', clock(auto), auto['data[1]'])]),
            ('Horizontal speed m/s', [('actual', clock(lp), np.hypot(lp['vx'], lp['vy']))]),
            ('Down m/s: raw Land blue / output red / measured green', [
                ('raw', clock(auto), auto['data[12]']), ('output', clock(auto), auto['data[24]']),
                ('measured', clock(auto), auto['data[21]'])])], land, min(land+65, end-2))
        # Save compact interpolated data for independent/local-model review.
        grid = np.arange(front-18, end-2, .02)
        columns = [grid]
        headers = ['boot_s']
        for topic, keys in ((angular, [f'xyz[{i}]' for i in range(3)]), (rates, ['roll', 'pitch', 'yaw']),
                            (torque, [f'xyz[{i}]' for i in range(3)]), (lp, ['x','y','z','vx','vy','vz']),
                            (lps, ['x','y','z','vx','vy','vz'])):
            for key in keys:
                headers.append(str(len(columns))+'_'+key)
                columns.append(interp(topic, key, grid))
        truth_columns = {'truth_time': truth['time']-offset, 'thrust': truth['thrust'], 'power': truth['power']} if truth is not None else {}
        np.savez_compressed(output/'resampled.npz', data=np.array(columns).T, headers=headers, **truth_columns)
        (output/'metrics.json').write_text(json.dumps(metrics, indent=2, allow_nan=False)+'\n')
        return metrics


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('archive', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    result = analyze(args.archive, args.output)
    print(json.dumps({key: result[key] for key in ('events_boot_s', 'estimator_truth', 'truth')}, indent=2))
