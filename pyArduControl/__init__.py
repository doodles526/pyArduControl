import pyfirmata
from pyfirmata import Arduino, util

import threading

class ArduControl():

    def __init__(self, extension, control_board=None, encoder_board=None):
        if control_board is not None:
            self.board = control_board
        else:
            self.board = pyfirmata.Arduino(extension)

        self._motorA_pos_thread = threading.Thread()
        self._motorB_pos_thread = threading.Thread()

        self._motorA_hold = threading.Event()
        self._motorB_hold = threading.Event() 

        self.motorA_newpos = threading.Event()
        self.motorB_newpos = threading.Event()

        self.motorA_speed = self.board.get_pin("d:3:p")
        self.motorA_direction = self.board.get_pin("d:12:o")

        self.motorB_speed = self.board.get_pin("d:11:p")
        self.motorB_direction = self.board.get_pin("d:13:0")

        self.p_gain = .01
        self.d_gain = .01

        if encoder_board:
            import serial
            self.encoder = serial.Serial(encoder_board)

    def motorSpeed(self, motorA=None, motorB=None):
        if motorA is not None:
            self.motorA_speed.write(motorA)

        if motorB is not None:
            self.motorB_speed.write(motorB)

    def motorDirection(self, motorA=None, motorB=None):
        if motorA is not None:
            self.motorA_direction.write(motorA)

        if motorB is not None:
            self.motorB_direction.write(motorB)

    def motorDirectionToggle(self, motorA=None, motorB=None):
        if motorA:
            self.motorA_direction.write(not self.motorA_direction.read())
        if motorB:
            self.motorA_direction.write(not self.motorA_direction.read())


    def gotoPosition(self, motorA=None, motorB=None):
        """
        Sends a motor to a given position.
        This takes a tuple of (revs, counts)
        Requires an encoder to function
        """

        if motorA:
            self._motorA_newpos.set()
            while not self._motorA_pos_thread.isAlive():
                pass
            self._motorA_pos_thread(motorAWorker, (motorA, ))
        if motorB:
            self._motorB_newpos.set()
            while not self._motorB_pos_thread.isAlive():
                pass

            self._motorB_pos_thread(motorBWorker, (motorB, ))

    def motorAWorker(self, goto_pos):
        """
        Worker function for thread gotoPosition
        """
       
        while not self._motorA_newpos.isSet():
            self._motorA_hold.wait()
            self.motorDirection(motorA=(goto_pos-phy_pos > 0))
            p_vel = self.p_gain * (abs((goto_pos[0]*1856 + goto_pos[1]) - (phy_pos[0]*1856 + phy_pos[1])))
            d_vel = self.d_gain * self.motorA_speed.read()
            vel = p_vel
            self.motorSpeed(motorA=vel)
        self._motorA_newpos.clear()

    def motorBWorker(self, goto_pos):
        while not self._motorB_newpos.isSet():
            self._motorB_hold.wait()
            self.motorDirection(motorA=(goto_pos-phys_pos > 0))
            p_vel = self.p_gain * (abs((goto_pos[0]*1856 + goto_pos[1]) - (phy_pos[0]*1856 + phy_pos[1])))
            d_vel = self.d_gain * self.motorB_speed.read()
            vel = p_vel
            self.motorSpeed(motorB=vel)
        self._motorB_newpos.clear()
