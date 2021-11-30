#!/usr/bin/env python3

'''
Replays a Flightgear recording and shows framerate statistics.

Usage:
    -f <fgfs>
        Name of Flightear executable/script, e.g.: -f run_fgfs.sh
    -i <tape>
        Name of recording, for use with --load-tape.
'''

import math
import sys
import time

import recordreplay


def average_stddev(items):
    '''
    Returns (average, stddev).
    '''
    total = 0
    total_sq = 0
    for item in items:
        total += item
        total_sq += item*item
    n = len(items)
    average = total / n
    variance = total_sq / n - (total / n)**2
    stddev = math.sqrt(variance)
    return average, stddev


if __name__ == '__main__':
    fgfs = None
    tape = None
    args = iter(sys.argv[1:])
    while 1:
        try:
            arg = next(args)
        except StopIteration:
            break
        if arg in ('-h', '--help'):
            print(__doc__)
        elif arg == '-f':
            fgfs = next(args)
        elif arg == '-i':
            tape = next(args)
        else:
            raise Exception(f'Unrecognised arg: {arg}')
    
    if not fgfs:
        raise Exception(f'Specify fgfs executable/run-script with -f')
    if not tape:
        raise Exception(f'Specify tape to replay with -i')

    fg = recordreplay.Fg( None,
            f'{fgfs}'
                f' --load-tape={tape}'
                f' --timeofday=noon'
                f' --prop:bool:/sim/replay/log-frame-times=true'
                f' --prop:bool:/sim/replay/replay-main-view=true'
                f' --prop:bool:/sim/replay/replay-main-window-size=true'
                f' --prop:bool:/sim/replay/looped=false'
            )

    fg.waitfor('/sim/fdm-initialized', 1, timeout=45)
    fg.waitfor('/sim/replay/replay-state', 1)
    fg.waitfor('/sim/replay/replay-state-eof', 1, timeout=600)
    
    print(f'Reading frame-time statistics...')
    t = time.time()
    items = fg.fg.ls('/sim/replay/log-frame-times')
    t = time.time() - t
    print(f'fg.fg.ls took t={t}')
    
    fg.close()
    
    dts = []
    for item in items:
        if item.name == 'dt':
            dts.append(float(item.value))
    
    def statistics_text(dts):
        dt_average, dt_stddev = average_stddev(dts)
        t_total = sum(dts)
        return f'n={len(dts)} dt_average={dt_average} dt_stddev={dt_stddev} t_total={t_total} fps={len(dts)/t_total}'
    
    print(f'-' * 40)
    print(f'')
    print(f'Overall frame time (dt) statistics:')
    print(f'    {statistics_text(dts)}')
    
    print(f'Ignoring first 4 frames:')
    print(f'    {statistics_text(dts[4:])}')
    
    print(f'')
    print(f'Raw frame times:')
    dts_text = ' '.join(map(lambda dt: f'{dt:.4f}', dts))
    print(f'dts: {dts_text}')
