"""Run the existing PF survey validator alongside RViz, and clean up both."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys

PROJECT = Path(__file__).resolve().parents[2]
OUTPUT = Path(__file__).resolve().parent
env = {**os.environ, 'ROS_DOMAIN_ID': '181',
       'ROS_LOG_DIR': str(OUTPUT / 'rviz_ros_log')}
rviz_command = ['ros2', 'run', 'rviz2', 'rviz2', '-d',
                str(PROJECT / 'install/baseline_robot_description/share/'
                    'baseline_robot_description/config/config.rviz')]
mission_command = [sys.executable, str(PROJECT / 'scripts/validate_particle_navigation.py'),
                   '--output-dir', str(OUTPUT / 'survey'), '--domain-id', '181']
status = {'rviz_command': rviz_command, 'mission_command': mission_command}
with (OUTPUT / 'rviz.log').open('x') as log:
    rviz = subprocess.Popen(rviz_command, env=env, stdout=log, stderr=log,
                            start_new_session=True)
    try:
        result = subprocess.run(mission_command, env=env)
        status['mission_exit_code'] = result.returncode
        status['rviz_alive_at_mission_end'] = rviz.poll() is None
    finally:
        if rviz.poll() is None:
            os.killpg(rviz.pid, signal.SIGINT)
        try:
            rviz.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(rviz.pid, signal.SIGTERM)
            rviz.wait(timeout=5)
        status['rviz_exit_code'] = rviz.returncode
        (OUTPUT / 'runtime_status.json').write_text(json.dumps(status, indent=2) + '\n')
sys.exit(status['mission_exit_code'] or (not status['rviz_alive_at_mission_end']))
