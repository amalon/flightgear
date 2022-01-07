#!/usr/bin/env python3

'''
Test script for video encoding.

Example usage:
    scripts/python/video.py -f dac/run_fgfs.sh

'''

import recordreplay

import os
import sys
import time

def main():
    fgfs = f'./build-walk/fgfs.exe-run.sh'
    
    args = iter(sys.argv[1:])
    while 1:
        try:
            arg = next(args)
        except StopIteration:
            break
        if arg == '-f':
            fgfs = next(args)
        else:
            raise Exception(f'Unrecognised arg: {arg}')
    
    fg = recordreplay.Fg(
            'harrier-gr3',
            f'{fgfs}'
                + f' --state=vto --airport=egtk'
                + f' --prop:/sim/replay/record-main-view=1'
                + f' --prop:bool:/sim/replay/record-main-window=0'
                ,
            )

    fg.waitfor('/sim/fdm-initialized', 1, timeout=45)
    fg.fg['/sim/current-view/view-number-raw'] = 1 # helicopter

    # Rotation speed.
    fg.fg['/controls/auto-hover/rotation-speed-target'] = 4
    fg.fg['/controls/auto-hover/rotation-mode'] = 'speed'

    # These will have been set by --state=vto
    # Sideways speed.
    fg.fg['/controls/auto-hover/x-speed-target'] = '0'
    fg.fg['/controls/auto-hover/x-mode'] = 'speed'

    # Vertical speed.
    fg.fg['/controls/auto-hover/y-speed-target'] = '0'
    fg.fg['/controls/auto-hover/y-mode'] = 'speed'

    # Forwards speed.
    fg.fg['/controls/auto-hover/z-speed-target'] = '0'
    fg.fg['/controls/auto-hover/z-mode'] = 'speed'

    results = []
    
    def make_recording(codec, container, quality, speed, fixed_dt=None):
        '''
        Create recording using specified codec etc.
        '''
        fg.fg['/sim/video/container'] = container
        fdm_time_begin = fg.fg['/sim/time/simple-time/fdm']
        if fixed_dt:
            fg.fg['/sim/time/simple-time/enabled'] = True
            fg.fg['/sim/time/fixed-dt'] = fixed_dt
        name = f'video-test-c={codec}-q={quality}-s={speed}.{container}'
        frames_start = fg.fg['/sim/frame-number']
        fg.run_command( f'run video-start name={name} quality={quality} speed={speed} codec={codec}')
        dt = 10
        time.sleep(dt)
        frames = fg.fg['/sim/frame-number'] - frames_start
        fdm_time_end = fg.fg['/sim/time/simple-time/fdm']
        fg.run_command( f'run video-stop')
        fdm_time = fdm_time_end - fdm_time_begin
        e = fg.fg['/sim/video/error']
        e = f'error e={e:3}' if e else 'success'
        e = f'{e:12}'
        if not e:
            if not os.path.isfile(name):
                e = 'error no output file'
        result = f'Video encoding result: {e}: codec={codec:12} container={container:6} quality={quality:6} speed={speed:6} frames={frames:6} frame_rate={frames/dt:6} fdm_time={fdm_time:6} size={os.path.getsize(name):9}: {name}'
        results.append( result)
        print( result)
    
    if 1:
        # Create Continuous recording, replay and create video, check video is
        # new.
        
        # Create Continuous recording.
        video_suffix = 'mkv'
        tstart = time.time()
        fg.fg['/sim/video/container'] = video_suffix
        fg.fg['/sim/video/codec'] = 'libx265'
        fg.fg['/sim/video/quality'] = 0.75
        fg.fg['/sim/video/speed'] = 1.0
        fg.fg['/sim/replay/record-continuous-compression'] = 1
        fg.fg['/sim/replay/record-continuous'] = 1
        fg.fg['/sim/replay/record-main-view'] = 1
        endtime = tstart + 10
        while 1:
            if time.time() > endtime:
                break
            time.sleep(1)
            fg.run_command('run view-step step=1')
        fg.fg['/sim/replay/record-continuous'] = 0
        
        # Replay to create video.
        tstart = time.time()
        fg.fg['sim/replay/replay-main-view'] = 1
        fg.fg['sim/replay/replay-windows-position'] = 0
        fg.fg['sim/replay/replay-windows-size'] = 0
        fg.fg['/sim/video/container'] = 'mkv'
        fg.fg['/sim/video/codec'] = 'libx265'
        fg.fg['/sim/video/quality'] = 0.75
        fg.fg['/sim/video/speed'] = 1.0
        fg.run_command( f'run load-tape tape={fg.aircraft}-continuous create-video=1 fixed-dt=0.04')
        fg.waitfor('/sim/replay/replay-state', 1)   # Wait for replay to start.
        fg.waitfor('/sim/replay/replay-state-eof', 1)   # Wait for replay eof.
        
        # Check video looks ok.
        video_path = f'fgvideo-harrier-gr3.{video_suffix}'
        video_path2 = recordreplay.readlink( video_path)
        print(f'*** video_path={video_path} video_path2={video_path2}')
        t = os.path.getmtime(video_path2)
        assert t > tstart, f'Video file too old: {video_path2}'
        
    elif 1:
        make_recording('libtheora', 'ogv', quality=1, speed=1, fixed_dt=0.02)
        make_recording('libx265', 'mkv', quality=1, speed=1, fixed_dt=0.02)
    
    else:
        for codec in (
                'libtheora',
                'libx265',
                'mpeg2video',
                'libx264',
                'libvpx',
                ):
            fg.fg['/sim/video/codec'] = codec
            for container in (
                    'mpeg',
                    'ogv',
                    'mkv',
                    ):
                # High quality can semi-freeze Flightgear.
                for quality, speed in [
                        ('0.4', '0.5'),
                        ('0.4', '0.9'),
                        ('0.1', '0.5'),
                        ('0.1', '0.9'),
                        ]:
                    # This ordering should give increasing frame rate.
                    make_recording(codec, container, quality, speed)
            
    print('Results:')
    for result in results:
        print( f'    {result}')

if __name__ == '__main__':
    main()
