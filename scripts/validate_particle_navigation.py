#!/usr/bin/env python3
"""Launch an isolated Stonefish/PF survey, record metrics, and stop all owned nodes.

Run after `source scripts/env.sh`. The output directory must not already exist.
No physical robot topics are used: the run gets its own ROS_DOMAIN_ID.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import SetBool
from mvp_msgs.srv import ChangeState


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', required=True, type=Path)
    parser.add_argument('--use-usbl', choices=['true', 'false'], default='true')
    parser.add_argument('--domain-id', type=int, default=181)
    args = parser.parse_args()
    if not 1 <= args.domain_id <= 232:
        parser.error('choose an isolated domain in [1,232]')
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=False)
    env = {**os.environ, 'ROS_DOMAIN_ID': str(args.domain_id), 'ROS_LOG_DIR': str(output/'ros_log')}
    command = ['ros2', 'launch', 'baseline_robot_bringup', 'bringup_simulation.launch.py',
               'rviz:=false', 'localization_filter:=pf', f'use_usbl:={args.use_usbl}',
               f'csv_path:={output / "trajectory.csv"}']
    log = (output/'launch.log').open('x')
    rclpy.init(domain_id=args.domain_id)
    node = rclpy.create_node('paper_pf_mission_validation')
    estimates, references, diagnostics, states = [], [], [], []
    control = node.create_client(SetBool, '/baseline_robot/controller/set')
    mission = node.create_client(ChangeState, '/baseline_robot/mvp_helm/change_state')
    subscriptions = [
        node.create_subscription(Odometry, '/baseline_robot/odometry/filtered',
                                 lambda msg: estimates.append((time.monotonic(), msg)), 100),
        node.create_subscription(Odometry, '/baseline_robot/gps/ground_truth/odometry',
                                 lambda msg: references.append(msg), 100),
        node.create_subscription(DiagnosticArray, '/baseline_robot/navigation/pf_diagnostics',
                                 lambda msg: diagnostics.append(msg), 100),
        node.create_subscription(String, '/baseline_robot/mvp_helm/current_helm_state',
                                 lambda msg: states.append((time.monotonic(), msg.data)), 100),
    ]
    process = subprocess.Popen(command, env=env, stdout=log, stderr=log, start_new_session=True)

    def spin(seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            if process.poll() is not None:
                raise RuntimeError('launch exited; see launch.log')
            rclpy.spin_once(node, timeout_sec=.05)

    def call(client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.)
        if not future.done() or future.result() is None:
            raise RuntimeError('service call timed out')
        return future.result()

    summary = {'command': command, 'use_usbl': args.use_usbl == 'true', 'status': 'failed'}
    started = False
    try:
        deadline = time.monotonic()+35
        while not (estimates and references and diagnostics and control.service_is_ready()
                   and mission.service_is_ready()):
            spin(.1)
            if time.monotonic() > deadline:
                raise RuntimeError('required navigation/services not ready')
        spin(8.)
        publishers = node.get_publishers_info_by_topic('/baseline_robot/odometry/filtered')
        summary['odometry_publishers'] = [row.node_name for row in publishers]
        assert len(publishers) == 1 and publishers[0].node_name == 'pf_filter_node'
        assert not any(name == 'ekf_filter_node' for name, _ in node.get_node_names_and_namespaces())
        assert call(control, SetBool.Request(data=True)).success
        started = True
        assert call(mission, ChangeState.Request(state='survey', caller='paper_pf_validation')).status
        begin, completed, seen_survey, reported = time.monotonic(), None, False, 0.
        while time.monotonic()-begin < 250:
            spin(.1)
            if states:
                seen_survey |= states[-1][1] == 'survey'
                if seen_survey and states[-1][1] == 'start' and completed is None:
                    completed = time.monotonic()
            if time.monotonic()-reported > 15:
                p = estimates[-1][1].pose.pose.position
                print(f'{args.use_usbl=} elapsed={time.monotonic()-begin:.1f}s '
                      f'state={states[-1][1] if states else "?"} '
                      f'position=({p.x:.2f},{p.y:.2f},{p.z:.2f})', flush=True)
                reported = time.monotonic()
            if completed is not None and time.monotonic()-completed >= 12:
                break
        else:
            raise RuntimeError('rectangle did not complete within 250 seconds')
        assert call(control, SetBool.Request(data=False)).success
        started = False
        xs = [row.pose.pose.position.x for row in references]
        ys = [row.pose.pose.position.y for row in references]
        assert max(xs) > 15 and max(ys) > 15
        diag = [{kv.key: kv.value for kv in row.status[0].values} for row in diagnostics if row.status]
        stamps = [msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9 for _, msg in estimates]
        assert all(b > a for a, b in zip(stamps, stamps[1:]))
        summary.update(status='complete', mission_duration_sec=completed-begin,
                       final_state=states[-1][1], samples=len(estimates),
                       output_hz=(len(stamps)-1)/(stamps[-1]-stamps[0]),
                       diagnostics=diag, reference_extents={'max_x': max(xs), 'max_y': max(ys)})
    except Exception as exc:
        summary['error'] = str(exc)
        raise
    finally:
        if started and control.service_is_ready():
            try:
                call(control, SetBool.Request(data=False))
            except Exception:
                pass
        (output/'mission.json').write_text(json.dumps(summary, indent=2)+'\n')
        node.destroy_node()
        rclpy.shutdown()
        if process.poll() is None:
            # Signal launch once; it forwards SIGINT to its children. Signalling
            # the entire group as well causes a second interrupt during cleanup.
            process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=5)
        log.close()
    root = Path(__file__).resolve().parents[1]
    evaluator = [sys.executable, str(root/'baseline_robot_bringup/scripts/evaluate_navigation.py'),
                 str(output/'trajectory.csv'), '--json', str(output/'metrics.json')]
    if args.use_usbl == 'true':
        evaluator += ['--profile', 'usbl', '--check']
    result = subprocess.run(evaluator, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    evaluation_text = result.stdout
    if (output/'metrics.json').exists():
        # The legacy evaluator's profile checks 3D RMSE. This migration also
        # explicitly requires a bound on MAXIMUM 3D error, a different metric.
        path = output/'metrics.json'
        metrics = json.loads(path.read_text())
        metrics['acceptance_checked'] = args.use_usbl == 'true'
        if args.use_usbl == 'true':
            metrics['pf_acceptance_limits_m'] = {'horizontal_rmse_m': .30,
                                                  'max_error_3d_m': .35,
                                                  'final_error_3d_m': .35}
            metrics['pf_passed'] = all(metrics[key] <= limit
                                       for key, limit in metrics['pf_acceptance_limits_m'].items())
        else:
            metrics['profile'] = 'report_only'
            metrics.pop('passed', None)
            metrics.pop('acceptance_limits_m', None)
        path.write_text(json.dumps(metrics, indent=2)+'\n')
        evaluation_text = json.dumps(metrics, indent=2)+'\n'
        (output/'evaluation.log').write_text(evaluation_text)
        print(evaluation_text, flush=True)
        if args.use_usbl == 'true' and not metrics['pf_passed']:
            return 1
    else:
        (output/'evaluation.log').write_text(evaluation_text)
        print(evaluation_text, flush=True)
    return result.returncode


if __name__ == '__main__':
    raise SystemExit(main())
