#!/usr/bin/env python 

#****************************
# cfv2_gamepad.py
# Librería de funciones para leer datos desde un joystick genérico tipo Xbox.
# Requiere la librería inputs (pip install inputs)
# Código fuente e instrucciones de uso: http://github.com/cear-inacap/york-control
#
# Autor: Cesar Fuenzalida @cefuve
# Versión: 1.0 - enero 2024
#*****************************

from inputs import get_gamepad
import math
import threading

joy_cal = 128

class cfv_gamepad(object):
    MAX_TRIG_VAL = math.pow(2, 8)
    MAX_JOY_VAL = math.pow(2, 15)

    def __init__(self):

        self.LeftJoystickY = joy_cal
        self.LeftJoystickX = joy_cal
        self.RightJoystickY = joy_cal
        self.RightJoystickX = joy_cal
        self.LeftBrake = 0
        self.RightGas = 0
        
        self.LeftTrigger = 0
        self.RightTrigger = 0
        self.LeftBumper = 0
        self.RightBumper = 0
        self.A = 0
        self.X = 0
        self.Y = 0
        self.B = 0
        self.LeftThumb = 0
        self.RightThumb = 0
        self.Back = 0
        self.Start = 0
        self.LeftDPad = 0
        self.RightDPad = 0
        self.UpDPad = 0
        self.DownDPad = 0

        self._monitor_thread = threading.Thread(target=self._monitor_controller, args=())
        self._monitor_thread.daemon = True
        self._monitor_thread.start()


    def read(self): 
        # aguegar aquí los controles del joystick
        # que resulten de interés
        x = self.LeftJoystickX
        y = self.LeftJoystickY
        a = self.A
        b = self.B
        brake = self.LeftBrake
        gas = self.RightGas
        x2 = self.RightJoystickX
        y2 = self.RightJoystickY
        return [x, y, x2, y2, a, b]


    def _monitor_controller(self):
        while True:
            events = get_gamepad()
            for event in events:
                #print(event.code)
                if event.code == 'ABS_Y':
                    self.LeftJoystickY = event.state
                elif event.code == 'ABS_X':
                    self.LeftJoystickX = event.state
                elif event.code == 'ABS_RZ':
                    self.RightJoystickY = event.state
                elif event.code == 'ABS_Z':
                    self.RightJoystickX = event.state
                elif event.code == 'ABS_BRAKE':
                    self.LeftBrake = event.state
                elif event.code == 'ABS_GAS':
                    self.RightGas = event.state
                elif event.code == 'ABS_Z':
                    self.LeftTrigger = event.state
                elif event.code == 'ABS_RZ':
                    self.RightTrigger = event.state
                elif event.code == 'BTN_TL':
                    self.LeftBumper = event.state
                elif event.code == 'BTN_TR':
                    self.RightBumper = event.state
                elif event.code == 'BTN_SOUTH':
                    self.A = event.state
                elif event.code == 'BTN_NORTH':
                    self.Y = event.state #previously switched with X
                elif event.code == 'BTN_WEST':
                    self.X = event.state #previously switched with Y
                elif event.code == 'BTN_EAST':
                    self.B = event.state
                elif event.code == 'BTN_THUMBL':
                    self.LeftThumb = event.state
                elif event.code == 'BTN_THUMBR':
                    self.RightThumb = event.state
                elif event.code == 'BTN_SELECT':
                    self.Back = event.state
                elif event.code == 'BTN_START':
                    self.Start = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY1':
                    self.LeftDPad = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY2':
                    self.RightDPad = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY3':
                    self.UpDPad = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY4':
                    self.DownDPad = event.state


if __name__ == '__main__':
    joy = cfv_gamepad()
    while True:
        #pass
        print(joy.read())