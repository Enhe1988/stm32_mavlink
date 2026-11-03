"""Host C regression tests; requires a native GCC (not arm-none-eabi-gcc)."""
from pathlib import Path
import os
import subprocess

root = Path(__file__).resolve().parents[2]
test = root / 'tests/imu'
output = root / 'build/imu-host'
output.mkdir(parents=True, exist_ok=True)
exe = output / ('test_imu.exe' if os.name == 'nt' else 'test_imu')
sources = [test / 'test_imu.c', *[root / 'bsp/imu' / name for name in
           ['imu_api.c', 'icm_acq.c', 'icm42688p.c', 'icm45686.c', 'imu_frame.c']]]
count = 0
for model in range(1, 4):
    subprocess.run([os.environ.get('HOST_CC', 'gcc'), '-std=c11', '-Wall', '-Wextra',
                    '-Werror', '-Wstrict-prototypes', f'-DAIRCRAFT_MODEL={model}',
                    '-I' + str(test / 'stubs'), '-I' + str(root / 'bsp/imu'),
                    '-I' + str(root / 'bsp'), *map(str, sources),
                    '-lm', '-o', str(exe)], check=True)
    for mask in range(8):
        for failures in range(8):
            for task_fail in range(2):
                subprocess.run([str(exe), str(mask), str(failures), str(task_fail)],
                               check=True, stdout=subprocess.PIPE)
                count += 1
print(f'{count} scenarios passed: selection/fallback, initialization, decoding, '
      'DMA buffer ownership, invalid samples, snapshot transforms, and error recovery.')
