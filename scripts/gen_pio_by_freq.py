# ------------------------------------------------------
# Copyright (c) 2023, Elehobica
# Released under the BSD-2-Clause
# refer to https://opensource.org/licenses/BSD-2-Clause
# ------------------------------------------------------

import os
import sys

SYSTEM_CLOCK_FREQ = 125000000

in_file = sys.argv[1]
out_dir = sys.argv[2]
frequencies = sys.argv[3:]

if not os.path.isdir(out_dir):
    os.makedirs(out_dir)

with open(in_file, 'r') as rf:
    text_org = rf.read()
    for freq in frequencies:
        out_file = f'{out_dir}/{os.path.splitext(os.path.basename(in_file))[0]}_{freq}.pio'
        cycles = int(SYSTEM_CLOCK_FREQ / (int(freq) * 128) + 0.5)
        text = text_org
        text = text.replace('48000', freq)
        text = text.replace('define cy 20', f'define cy {cycles}')
        with open(out_file, 'w') as wf:
            wf.write(text)
