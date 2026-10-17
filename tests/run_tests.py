"""Portable runner for the same firmware logic matrix as run_tests.ps1."""
from pathlib import Path
import shutil
import subprocess
import os
import sys

root = Path(__file__).resolve().parents[1]
out = root / 'build' / 'host-tests'
out.mkdir(parents=True, exist_ok=True)
cxx = shutil.which('g++')
if not cxx:
    raise SystemExit('g++ is required')
base = [cxx, '-std=c++17', '-Wall', '-Wextra', '-Wpedantic',
        '-Itests/hal_stubs', '-IC_board_src']
sources = ['tests/control_framework_test.cpp', 'C_board_src/ControlTask.cpp',
           'C_board_src/Gm6020.cpp', 'C_board_src/Pid.cpp', 'C_board_src/AssessmentController.cpp']
cases = {
    'default-can1': [], 'derivative-disabled': ['CONTROL_ASSESSMENT_SPEED_KD=0.0f'],
    'angle-ff-disabled': ['CONTROL_ASSESSMENT_ANGLE_FF_ENABLED=0'],
    'voltage-id1': ['CONTROL_COMMAND_CURRENT=0'],
    'voltage-id5': ['CONTROL_COMMAND_CURRENT=0', 'CONTROL_MOTOR_ID=5'],
    'transmit-disabled': ['CONTROL_CAN_TX_ENABLED=0'],
    'exact-id-filter': ['CONTROL_CAN_DIAGNOSTIC_RX=0'],
    'default-can2': ['CONTROL_CAN_CHANNEL=2'],
    'test-pi': ['CONTROL_SPEED_KP=4.0f', 'CONTROL_SPEED_KI=1.0f', 'CONTROL_SPEED_OUTPUT_LIMIT=5.0f'],
    'calculation-disabled': ['CONTROL_SPEED_CALC_ENABLED=0', 'CONTROL_SPEED_KP=4.0f'],
    'tx-can1-id1': ['CONTROL_CAN_TX_ENABLED=1', 'CONTROL_SPEED_KP=4.0f', 'CONTROL_SPEED_KI=1.0f', 'CONTROL_SPEED_OUTPUT_LIMIT=5.0f'],
    'tx-can2-id5': ['CONTROL_CAN_TX_ENABLED=1', 'CONTROL_CAN_CHANNEL=2', 'CONTROL_MOTOR_ID=5'],
    'tx-id4': ['CONTROL_MOTOR_ID=4'], 'tx-id7': ['CONTROL_MOTOR_ID=7'],
    'tx-calculation-disabled': ['CONTROL_CAN_TX_ENABLED=1', 'CONTROL_SPEED_CALC_ENABLED=0']
}
for name, flags in cases.items():
    exe = out / (name + ('.exe' if os.name == 'nt' else ''))
    subprocess.run(base + ['-D' + x for x in flags] + sources + ['-o', str(exe)], cwd=root, check=True)
    subprocess.run([str(exe)], cwd=root, check=True)
exe = out / ('input-control.exe' if os.name == 'nt' else 'input-control')
subprocess.run(base + ['tests/input_control_test.cpp', 'C_board_src/SerialCommand.cpp',
                       'C_board_src/KeyControl.cpp', '-o', str(exe)], cwd=root, check=True)
subprocess.run([str(exe)], cwd=root, check=True)
subprocess.run([sys.executable, 'tests/test_export_assessment.py'], cwd=root, check=True)
