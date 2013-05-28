import pyfirmata
from pyfirmata import Arduino, util
import serial
import time
from multiprocessing import Process, Lock
from multiprocessing.sharedctypes import Array
from ctypes import Structure, c_long
import re

class ArduControl():

    def __init__(self, extension, control_board=None, encoder_board=None):
        if control_board is not None:
            self.board = control_board
        else:
            self.board = pyfirmata.Arduino(extension)

        self._motorA_pos_process = None
        self._motorB_pos_process = None 

        self.motorA_speed = self.board.get_pin("d:3:p")
        self.motorA_direction = self.board.get_pin("d:12:o")
        self.motorA_brake = self.board.get_pin("d:9:o")

        self.motorB_speed = self.board.get_pin("d:11:p")
        self.motorB_direction = self.board.get_pin("d:13:o")
        self.motorB_brake = self.board.get_pin("d:8:o")

        self._triggerA_process = None
        self._triggerB_process = None

        self.triggerA = self.board.digital[6]
        self.triggerB = self.board.digital[7]

        self.board.servo_config(pin=9, min_pulse=600, max_pulse=2400, angle=35)
        self.board.servo_config(pin=10, min_pulse=600, max_pulse=2400, angle = 145)

        self.trigger_time_constant = .5

        self.p_gain = .01
        self.d_gain = 2

        self.encoder = encoder_board
        self.motorSpeed(0, 0)
        if encoder_board:
            self.encoder = encoder_board



    def fire(self, triggerA=None, triggerB=None):
        if triggerA is not None:
            if self._triggerA_process:
                self._triggerA_process.terminate()
                while self._triggerA_process.is_alive():
                    pass
            self._triggerA_process = Process(target=self.triggerAWorker, args=(triggerA,))
            self._triggerA_process.start()
        if triggerB is not None:
            if self._triggerB_process:
                self._triggerB_process.terminate()
                while self._triggerB_process.is_alive():
                    pass
            self._triggerB_process = Process(target=self.triggerBWorker, args=(triggerA,))
            self._triggerB_process.start()


    def triggerAWorker(self, num_fire):
        self.triggerA.write(0)
        time.sleep(num_fire * self.trigger_time_constant)
        self.triggerA.write(25)
    def triggerBWorker(self, num_fire):
        self.triggerB.write(180)
        time.sleep(num_fire * self.trigger_time_constant)
        self.triggerB.write(155)

    def brake(self, motorA=None, motorB=None):
        if motorA is not None:
            self.motorA_brake.write(1)

        if motorB is not None:
            self.motorB_brake.write(1)

    def haltMotor(self, motorA=None, motorB=None):
        if motorA:
            if self._motorA_pos_process and self._motorA_pos_process.is_alive():
                self._motorA_pos_process.terminate()
            self.motorSpeed(motorA=1)
            self.motorSpeed(motorA=0)
        if motorB:
            if self._motorB_pos_process and self._motorB_pos_process.is_alive():
                self._motorB_pos_process.terminate()
            self.motorSpeed(motorB=0)

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
            self.motorB_direction.write(not self.motorB_direction.read())


    def gotoPosition(self, motorA=None, motorB=None):
        """
        Sends a motor to a given position.
        This takes a tuple of (revs, counts)
        Requires an encoder to function
        """

        if motorA:
            if self._motorA_pos_process:
                self._motorA_pos_process.terminate()
                while self._motorA_pos_process.is_alive():
                    pass
            self._motorA_pos_process = Process(target=self.motorAWorker, args=(motorA,))
            self._motorA_pos_process.start()
        if motorB:
            self._motorB_newpos.set()
            while not self._motorB_pos_thread.isAlive():
                pass
            self._motorB_pos_thread(motorBWorker, (motorB, ))

    def motorAWorker(self, goto_pos):
        """
        Worker function for thread gotoPosition
        """
       
        while True:
            phy_pos = (self.encoder.getPositions()[0])
            self.motorDirection(motorA=int(goto_pos < phy_pos))
            p_vel = self.p_gain * ((goto_pos[0]*464 + goto_pos[1]) - (phy_pos[0]*464 + phy_pos[1]))
            d_vel = self.d_gain * self.motorA_speed.read()
            if self.motorA_direction.read() == 1:
                d_vel = -d_vel
            vel = p_vel - d_vel
            if vel == abs(vel):
                self.motorDirection(0)
            else:
                self.motorDirection(1)
            vel = abs(vel)
            if vel > 1:
                vel = 1
            
            if self.motorA_speed.read() != vel:
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
    _fields_ = [('revs', c_long), ('counts', c_long)]


class Encoder(Process):
    
    def __init__(self, serial_ext, clicks_per_rev):
        Process.__init__(self)
        self.revolutions = 0
        self.clicks = 0
        self.click_per_rev = clicks_per_rev
        self._posLock = Lock()
        self.ser = serial.Serial(serial_ext)
        self.numMotors = len(self.ser.readline().split(';')[:-1])
        #self._posLock.acquire()
        self.positions = Array(Position, [(i, i) for i in range(self.numMotors)])
        #self._posLock.release()
        self.start()

    
    def convertPositionStdToTicks(self, position):
        return position[0]*self.clicks_per_rev + position[1]

    def convertPositionTicksToStd(self, position):
        return (position / self.clicks_per_rev, position % clicks_per_rev)

    def run(self):
        while True:
            motors = self.ser.readline()
            motors = motors.split("\n")[0]
            motors = motors.split(';')[:-1]
            temp_positions = list()
            positions_counter = 0
            for motor in motors:
                print motor
                if re.match(r"[-]?\d*[,][-]?\d*", motor):  # regex to eliminate bad data packets
                    self.positions[positions_counter].revs = int(motor.split(',')[0])
                    self.positions[positions_counter].counts = int(motor.split(',')[1])
                positions_counter += 1

    def getPositions(self):
        """
        API function so the user doesn't have to
        deal with weird ctype_arrays
        """

        retVal = [(motor.revs, motor.counts) for motor in self.positions]
        return retVal
