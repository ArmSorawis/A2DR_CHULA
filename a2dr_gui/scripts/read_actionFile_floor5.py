#!/usr/bin/env python

def selection_state_floor5(room_number):

    rotate2Room = 0
    rotate2Nextgoal = 0 

    soundPath = None
    sound_cycleTime = 0

    if room_number == "Start":
        soundPath = '/home/a2dr-robot/a2dr_ws/src/a2dr_voice/voice/5thFloor/move2machine.wav' # voice directory
        sound_cycleTime = 4 # second

    elif room_number == 1:
        rotate2Target = 60 # degree
        rotate2Nextgoal = 30 # degree

        soundPath = '/home/a2dr-robot/a2dr_ws/src/a2dr_voice/voice/5thFloor/machine.wav' # voice directory
        sound_cycleTime = 4 # second

    elif room_number == 2:
        rotate2Target = -90 # degree
        rotate2Nextgoal = -45 # degree

        soundPath = '/home/a2dr-robot/a2dr_ws/src/a2dr_voice/voice/5thFloor/room1.wav' # voice directory
        sound_cycleTime = 3 # second

    elif room_number == 3:
        rotate2Target = 135 # degree
        rotate2Nextgoal = -100 # degree

        soundPath = '/home/a2dr-robot/a2dr_ws/src/a2dr_voice/voice/5thFloor/room2.wav' # voice directory
        sound_cycleTime = 3 # second

    elif room_number == 4:
        rotate2Target = 125 # degree
        rotate2Nextgoal = 145 # degree

        soundPath = '/home/a2dr-robot/a2dr_ws/src/a2dr_voice/voice/5thFloor/room3.wav' # voice directory
        sound_cycleTime = 3 # second

    elif room_number == 5:
        rotate2Target = 179 # degree
        rotate2Nextgoal = 179 # degree

        soundPath = '/home/a2dr-robot/a2dr_ws/src/a2dr_voice/voice/5thFloor/room4.wav' # voice directory
        sound_cycleTime = 3 # second
    
    elif room_number == 6:
        rotate2Target = 179 # degree
        rotate2Nextgoal = 179 # degree

        soundPath = '/home/a2dr-robot/a2dr_ws/src/a2dr_voice/voice/5thFloor/room5.wav' # voice directory
        sound_cycleTime = 3 # second

    elif state == 'Finished':
        rotate2Target = 179 # degree

        soundPath = '/home/a2dr-robot/a2dr_ws/src/a2dr_voice/voice/5thFloor/base_station.wav' # voice directory
        sound_cycleTime = 5 # second


    selection_state_floor5.rotate2Target = rotate2Target
    selection_state_floor5.rotate2Nextgoal = rotate2Nextgoal

    selection_state_floor2.soundPath = soundPath
    selection_state_floor2.sound_cycleTime = sound_cycleTime
        
# selection_state_floor5('Finished')
# print(selection_state_floor5.rotate2Target)