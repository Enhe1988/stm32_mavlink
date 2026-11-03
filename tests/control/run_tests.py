"""Compile the actual mixer and installation transform for all three profiles."""
from pathlib import Path
import os
import subprocess

root = Path(__file__).resolve().parents[2]
output = root / 'build/control-host'
output.mkdir(parents=True, exist_ok=True)
cc = os.environ.get('HOST_CC', 'gcc')
for model in range(1, 4):
    exe = output / (f'profile{model}' + ('.exe' if os.name == 'nt' else ''))
    subprocess.run([cc, '-std=c11', '-Wall', '-Wextra', '-Werror',
                    f'-DAIRCRAFT_MODEL={model}', '-I' + str(root / 'algorithm'),
                    '-I' + str(root / 'bsp'), '-I' + str(root / 'bsp/imu'),
                    str(root / 'tests/control/test_profiles.c'),
                    str(root / 'algorithm/aircraft_mixer.c'), '-lm', '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
    attitude = output / (f'attitude{model}' + ('.exe' if os.name == 'nt' else ''))
    subprocess.run([cc, '-std=c11', '-Wall', '-Wextra', '-Werror',
                    f'-DAIRCRAFT_MODEL={model}', '-I' + str(root / 'tests/imu/stubs'),
                    '-I' + str(root / 'Core/Inc'), '-I' + str(root / 'algorithm'),
                    '-I' + str(root / 'bsp'), str(root / 'tests/control/test_attitude.c'),
                    str(root / 'algorithm/attitude_control.c'), str(root / 'algorithm/filter.c'),
                    str(root / 'algorithm/pid.c'),
                    str(root / 'algorithm/aircraft_mixer.c'), '-lm', '-o', str(attitude)], check=True)
    subprocess.run([str(attitude)], check=True)
# An invalid model must fail compilation instead of silently selecting a layout.
bad = subprocess.run([cc, '-std=c11', '-DAIRCRAFT_MODEL=99',
                      '-I' + str(root / 'bsp'), '-I' + str(root / 'algorithm'),
                      '-c', str(root / 'algorithm/aircraft_mixer.c'),
                      '-o', str(output / 'invalid.o')], capture_output=True)
assert bad.returncode != 0
print('Invalid model rejected at compile time')

filter_exe = output / ('filter' + ('.exe' if os.name == 'nt' else ''))
subprocess.run([cc, '-std=c11', '-Wall', '-Wextra', '-Werror',
                '-I' + str(root / 'algorithm'), str(root / 'tests/control/test_filter.c'),
                str(root / 'algorithm/filter.c'), '-lm', '-o', str(filter_exe)], check=True)
subprocess.run([str(filter_exe)], check=True)
for enabled in [0, 1]:
    mode_exe = output / (f'filter-mode{enabled}' + ('.exe' if os.name == 'nt' else ''))
    subprocess.run([cc, '-std=c11', '-Wall', '-Wextra', '-Werror',
                    f'-DCONTROL_GYRO_LPF_ENABLE={enabled}', '-DCONTROL_GYRO_LPF_DYNAMIC_ENABLE=0',
                    '-I' + str(root / 'tests/imu/stubs'), '-I' + str(root / 'Core/Inc'),
                    '-I' + str(root / 'algorithm'), '-I' + str(root / 'bsp'),
                    str(root / 'tests/control/test_filter_modes.c'),
                    str(root / 'algorithm/attitude_control.c'), str(root / 'algorithm/pid.c'),
                    str(root / 'algorithm/filter.c'), '-lm', '-o', str(mode_exe)], check=True)
    subprocess.run([str(mode_exe)], check=True)
