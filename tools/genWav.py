# -*- coding: utf-8 -*-

import sys
import re
import wave
import struct

def reverse_mask(x):
    x = ((x & 0x55555555) << 1) | ((x & 0xAAAAAAAA) >> 1)
    x = ((x & 0x33333333) << 2) | ((x & 0xCCCCCCCC) >> 2)
    x = ((x & 0x0F0F0F0F) << 4) | ((x & 0xF0F0F0F0) >> 4)
    x = ((x & 0x00FF00FF) << 8) | ((x & 0xFF00FF00) >> 8)
    x = ((x & 0x0000FFFF) << 16) | ((x & 0xFFFF0000) >> 16)
    return x

def main(filename):
    data = []
    for line in open(filename, "r"):
        if m := re.search(r'L \= ([\da-f]{4}), R \= ([\da-f]{4})', line, re.IGNORECASE):
            left = int(m.group(1), 16)
            right = int(m.group(2), 16)
            #print(f'{left = :#06x} {right = :#06x}')
            #print(f'{left = :016b} {right = :016b}')
            left = reverse_mask(left) >> 16
            right = reverse_mask(right) >> 16
            #print(f'{left = :#06x} {right = :#06x}')
            #print(f'{left = :016b} {right = :016b}')
            left -= 32768
            right -= 32768
            data.append(left)
            data.append(right)
    bin = struct.pack('h' * len(data), *data)

    print(len(data))

    wav = wave.Wave_write('out.wav')
    wav.setparams((
        2,         # channels
        2,         # byte width
        44100,     # sampling frequency
        len(data), # number of frames
        "NONE", "NONE" # no compression
    ))
    wav.writeframesraw(bin)
    wav.close()
    

if __name__ == "__main__":
    filename = sys.argv[1]
    main(filename)
