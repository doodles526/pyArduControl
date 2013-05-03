import pyfirmata
from pyfirmata import Arduino, util
import serial
from multiprocessing import Process
from multiprocessing.sharedctypes import Array
from ctypes import Structure, c_int

class ArduControl():

    def __init__(self, extension, control_board=None, encoder_board=None):
        if control_board is not None:
            self.board = control_board
        else:
            self.board = pyfirmata.Arduino(extension)

        self._motorA_pos_process = multiprocessing.Process()
        self._motorB_pos_process = multiprocessing.Process()

        self.motorA_speed = self.board.get_pin("d:3:p")
        self.motorA_direction = self.board.get_pin("d:12:o")

        self.motorB_speed = self.board.get_pin("d:11:p")
        self.motorB_direction = self.board.get_pin("d:13:0")

 

        self.p_gain = .01
        self.d_gain = .2

        self.encoder = encoder_board

        if encoder_board:
            self.encoder = encoder_board

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
            self._motorA_pos_process.terminate()
            while not self._motorA_pos_thread.isAlive():
                pass
            self._motorA_pos_process(motorAWorker, (motorA, ))
        if motorB:
            self._motorB_newpos.set()
            while not self._motorB_pos_thread.isAlive():
                pass

            self._motorB_pos_thread(motorBWorker, (motorB, ))

    def motorAWorker(self, goto_pos):
        """
        Worker function for thread gotoPosition
        """
       
        #while not self._motorA_newpos.isSet():
        while True:
            #self._motorA_hold.wait()
            phy_pos = self.encoder.getPositions()[0]
            self.motorDirection(motorA=int(goto_pos < phy_pos))
            p_vel = self.p_gain * (abs((goto_pos[0]*464 + goto_pos[1]) - (phy_pos[0]*464 + phy_pos[1])))
            d_vel = self.d_gain * self.motorA_speed.read()
            vel = abs(p_vel - d_vel)
            if vel > 1:
                vel = 1
            self.motorSpeed(motorA=vel)
        self._motorA_newpos.clear()

    def motorBWorker(self, goto_pos):
        while not self._motorB_newpos.isSet():
            self._motorB_hold.wait()
            self.motorDirection(motorA=int(goto_pos-phys_pos > 0))
            p_vel = self.p_gain * (abs((goto_pos[0]*1856 + goto_pos[1]) - (phy_pos[0]*1856 + phy_pos[1])))
            d_vel = self.d_gain * self.motorB_speed.read()
            vel = p_vel
            self.motorSpeed(motorB=vel)
        self._motorB_newpos.clear()

class Position(Structure):
    """
    Setup process-safe position list of tuples
    """
    _fields_ = [('revs', c_int), ('counts', c_int)]


class Encoder(Process):
    
    def __init__(self, serial_ext, clicks_per_rev):
        Process.__init__(self)
        self.revolutions = 0
        self.clicks = 0
        self.click_per_rev = 0
        self.ser = serial.Serial(serial_ext)
        self.numMotors = len(self.ser.readline().split(';'))
        self.positions = Array(Position, [(i, i) for i in range(self.numMotors)])

    def run(self):
        while True:
            motors = self.ser.readline()
            motors = motors.split("\n")[0]
            motors = motors.split(';')
            temp_positions = list()
            
            positions_counter = 0
            for motor in motors:
                self.positions[positions_counter].revs = int(motor.split(',')[0])
                self.positions[positions_counter].counts = int(motor.split(',')[1])
                positions_counter += 1

    def getPositions(self):
        """
        API function so the user doesn't have to
        deal with weird ctype_arrays
        """

        return [(motor.revs, motor.counts) for motor in self.positions]
