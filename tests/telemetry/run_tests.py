"""Compile production flow decoding, estimation and MAVLink telemetry with simulated INS/HAL."""
from pathlib import Path
import os
import subprocess
from generate_flow_vectors import write_vectors

root = Path(__file__).resolve().parents[2]
output = root / 'build/telemetry-host'
output.mkdir(parents=True, exist_ok=True)
write_vectors(output / 'flow_wire_vectors.h')
cc = os.environ.get('HOST_CC', 'gcc')
for model in range(1, 4):
    for swapped in [False, True]:
        exe = output / (f'telemetry-{model}-{int(swapped)}' + ('.exe' if os.name == 'nt' else ''))
        config = [f'-DAIRCRAFT_MODEL={model}']
        if swapped:
            config += ['-DFLOW_BODY_X_AXIS=1', '-DFLOW_BODY_Y_AXIS=0',
                       '-DFLOW_BODY_X_SCALE=-1.0f', '-DFLOW_BODY_Y_SCALE=2.0f']
        subprocess.run([cc, '-std=c11', '-Wall', '-Wextra', '-Werror',
                        '-Wno-address-of-packed-member', *config,
                        '-I' + str(root / 'tests/telemetry/stubs'),
                        '-I' + str(root / 'tests/imu/stubs'),
                        '-I' + str(root / 'application'), '-I' + str(root / 'algorithm'),
                        '-I' + str(root / 'bsp'), '-I' + str(root / 'Third_party/mavlink_c'),
                        str(root / 'tests/telemetry/test_telemetry.c'),
                        str(root / 'application/mavlink_flow.c'),
                        str(root / 'application/mavlink_telemetry.c'),
                        str(root / 'algorithm/optical_flow.c'),
                        '-lm', '-o', str(exe)], check=True)
        subprocess.run([str(exe)], check=True)
print('A/B/C telemetry passed with default and swapped/inverted/scaled flow axes')
tx_exe = output / ('tx' + ('.exe' if os.name == 'nt' else ''))
subprocess.run([cc, '-std=c11', '-Wall', '-Wextra', '-Werror',
                '-Wno-address-of-packed-member',
                '-I' + str(root / 'tests/telemetry/stubs'),
                '-I' + str(root / 'tests/imu/stubs'),
                '-I' + str(root / 'application'), '-I' + str(root / 'bsp'),
                '-I' + str(root / 'Third_party/mavlink_c'),
                str(root / 'tests/telemetry/test_tx.c'), '-lm', '-o', str(tx_exe)], check=True)
subprocess.run([str(tx_exe)], check=True)
rx_exe = output / ('rx' + ('.exe' if os.name == 'nt' else ''))
subprocess.run([cc, '-std=c11', '-Wall', '-Wextra', '-Werror',
                '-Wno-address-of-packed-member',
                '-I' + str(output),
                '-I' + str(root / 'tests/telemetry/stubs'),
                '-I' + str(root / 'tests/imu/stubs'),
                '-I' + str(root / 'application'), '-I' + str(root / 'algorithm'),
                '-I' + str(root / 'bsp'), '-I' + str(root / 'Third_party/mavlink_c'),
                str(root / 'tests/telemetry/test_rx.c'),
                str(root / 'application/mavlink_port.c'),
                str(root / 'application/mavlink_flow.c'),
                str(root / 'algorithm/optical_flow.c'),
                '-lm', '-o', str(rx_exe)], check=True)
subprocess.run([str(rx_exe)], check=True)
