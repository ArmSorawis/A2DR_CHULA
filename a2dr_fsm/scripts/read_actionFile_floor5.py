#!/usr/bin/env python

def selection_state_floor5(current_station, next_station):

    rotate2currentStation = 0
    rotate2nextStation = 0 

    soundPath = None
    sound_cycleTime = 0

    if current_station == "start":
        soundPath = '/home/ohm/a2dr_ws/src/a2dr_voice/voice/base2machine.wav' # voice directory
        sound_cycleTime = 4 # second

    elif current_station == "machine":
        rotate2currentStation = 60 # degree

        soundPath = '/home/ohm/a2dr_ws/src/a2dr_voice/voice/machine.wav' # voice directory
        sound_cycleTime = 4 # second
        
        if next_station == 1:
            rotate2nextStation = 90 # degree
        elif next_station == 5:
            rotate2nextStation = 179 # degree
        elif next_station == 2 or next_station == 3 or next_station == 4:
            rotate2nextStation = -90 # degree

    elif current_station == 1:
        rotate2currentStation = -90 # degree

        soundPath = '/home/ohm/a2dr_ws/src/a2dr_voice/voice/room1.wav' # voice directory
        sound_cycleTime = 3 # second
        
        if next_station == 'base':
            rotate2nextStation = 90 # degree
        elif next_station == 2 or next_station == 3 or next_station == 4 or next_station == 5:
            rotate2nextStation = -90 # degree

    elif current_station == 2:
        rotate2currentStation = 135 # degree

        soundPath = '/home/ohm/a2dr_ws/src/a2dr_voice/voice/room2.wav' # voice directory
        sound_cycleTime = 3 # second

        if next_station == 1:
            rotate2nextStation = 90 # degree
        elif next_station == 5:
            rotate2nextStation = 179 # degree
        elif next_station == 3 or next_station == 4:
            rotate2nextStation = -90 # degree
        elif next_station == 'base':
            rotate2nextStation = 90 # degree

    elif current_station == 3:
        rotate2currentStation = 125 # degree

        soundPath = '/home/ohm/a2dr_ws/src/a2dr_voice/voice/room3.wav' # voice directory
        sound_cycleTime = 3 # second

        if next_station == 4:
            rotate2nextStation = 179 # degree
        elif next_station == 1 or next_station == 2 or next_station == 5:
            rotate2nextStation = 90 # degree
        elif next_station == 'base':
            rotate2nextStation = 90 # degree

    elif current_station == 4:
        rotate2currentStation = 179 # degree

        soundPath = '/home/ohm/a2dr_ws/src/a2dr_voice/voice/room4.wav' # voice directory
        sound_cycleTime = 3 # second

        if next_station == 3:
            rotate2nextStation = 179 # degree
        elif next_station == 1 or next_station == 2 or next_station == 5:
            rotate2nextStation = -90 # degree
        elif next_station == 'base':
            rotate2nextStation = -90 # degree
    
    elif current_station == 5:
        rotate2currentStation = 179 # degree

        soundPath = '/home/ohm/a2dr_ws/src/a2dr_voice/voice/room5.wav' # voice directory
        sound_cycleTime = 3 # second

        if next_station == 1:
            rotate2nextStation = -90 # degree
        elif next_station == 2:
            rotate2nextStation = 179 # degree
        elif next_station == 3 or next_station == 4:
            rotate2nextStation = 90 # degree
        elif next_station == 'base':
            rotate2nextStation = -90 # degree

    elif current_station == 'base':
        soundPath = '/home/ohm/a2dr_ws/src/a2dr_voice/voice/base_station.wav' # voice directory
        sound_cycleTime = 5 # second

        rotate2nextStation = 179 # degree

    selection_state_floor5.rotate2currentStation = rotate2currentStation
    selection_state_floor5.rotate2nextStation = rotate2nextStation

    selection_state_floor5.soundPath = soundPath
    selection_state_floor5.sound_cycleTime = sound_cycleTime
        
# selection_state_floor5(1, 'base')
# print(selection_state_floor5.rotate2nextStation)