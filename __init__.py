import pyfirmata
from pyfirmata import Arduino, util
import serial
import time
from multiprocessing import Process, Lock, Event
from multiprocessing.sharedctypes import Array
from ctypes import Structure, c_long
import re


class MotorProcess(Process):
    """
    Process helper class for motor control allows for graceful exit
    """
    def __init__(self, control_funct, location):
        Process.__init__(self)
        self.exit = Event()
        self.control_funct = control_funct
        self.location = Position(location[0], location[1])

    def newPosition(self, pos):
        self.location = pos
        print self.location 
    def shutdown(self):
        self.exit.set()

    def run(self):
        while not self.exit.is_set():
            self.control_funct((self.location.revs, self.location.counts))

class TriggerProcess(Process):
    """
    Process helper class for triggering allows for graceful exit
    """
    def __init__(self, trigger_funct, num_fire):
        Process.__init__(self)
        self.exit = Event()
        self.trigger_funct = trigger_funct
        self.num_fire = num_fire

    def shutdown(self):
        self.exit.set()

    def run(self):
        while not self.exit.is_set:
            self.trigger_funct(self.num_fire)


class ArduControl():
    """
    Python library/API for controlling arduino motorcontrollers
    over USB serial connection.
    """
    def __init__(self, extension, encoder_board=None):
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
        self.triggerB = self.board.digital[10]

        self.board.servo_config(pin=6,
                                min_pulse=600,
                                max_pulse=2400,
                                angle=35)
        self.board.servo_config(pin=10,
                                min_pulse=600,
                                max_pulse=2400,
                                angle=145)

        self.trigger_time_constant = .5 * .2

        self.p_gain = .03
        self.d_gain = 5.2

        self.encoder = encoder_board
        self.motorSpeed(0, 0)
        if encoder_board:
            self.encoder = encoder_board

    def fire(self, triggerA=None, triggerB=None):
        """
        Master function and process starter for firing servos
        """
        if triggerA is not None:
            if self._triggerA_process:
                self._triggerA_process.shutdown()
                while self._triggerA_process.is_alive():
                    pass
            self._triggerA_process = TriggerProcess(self.triggerAWorker,
                                                    triggerA)
            self._triggerA_process.start()
        if triggerB is not None:
            if self._triggerB_process:
                self._triggerB_process.shutdown()
                while self._triggerB_process.is_alive():
                    pass
            self._triggerB_process = TriggerProcess(self.triggerBWorker,
                                                    triggerB)
            self._triggerB_process.start()

    def triggerAWorker(self, num_fire):
        """
        Worker function for fire.  This is specialized for crossfire, and
        should probably be abandoned for production.  But is fine for now.
        """
        self.triggerA.write(0)
        time.sleep(num_fire * self.trigger_time_constant)
        self.triggerA.write(25)

    def triggerBWorker(self, num_fire):
        """
        Worker function for fire.  This is specialized for crossfire, and
        should probably be abandoned for production.  But is fine for now.
        """
        self.triggerB.write(180)
        time.sleep(num_fire * self.trigger_time_constant)
        self.triggerB.write(155)

    def brake(self, motorA=None, motorB=None):
        """
        Uses braking functionality given in some arduino motor drivers
        """
        if motorA is not None:
            self.motorA_brake.write(1)

        if motorB is not None:
            self.motorB_brake.write(1)

    def haltMotor(self, motorA=None, motorB=None):
        """
        Stops a given motor, and all processes related to it
        """
        if motorA:
            if self._motorA_pos_process and \
               self._motorA_pos_process.is_alive():
                self._motorA_pos_process.terminate()
            self.motorSpeed(motorA=1)
            self.motorSpeed(motorA=0)
        if motorB:
            if self._motorB_pos_process and \
               self._motorB_pos_process.is_alive():
                self._motorB_pos_process.terminate()
            self.motorSpeed(motorB=0)

    def motorSpeed(self, motorA=None, motorB=None):
        """
        Sets motor to the given speed.  value must be between 0 and 1
        """
        if motorA is not None:
            if motorA < .6 and motorA != 0:
                motorA = .6
            self.motorA_speed.write(motorA)

        if motorB is not None:
            self.motorB_speed.write(motorB)

    def motorDirection(self, motorA=None, motorB=None):
        """
        sets the direction of the motor to given value
        """
        if motorA is not None:
            self.motorA_direction.write(motorA)

        if motorB is not None:
            self.motorB_direction.write(motorB)

    def motorDirectionToggle(self, motorA=None, motorB=None):
        """
        Switches the direction of the given motor
        """
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
                self._motorA_pos_process.shutdown()
                while self._motorA_pos_process.is_alive():
                    pass
            self._motorA_pos_process = MotorProcess(self.motorAWorker,
                                                    motorA)
            self._motorA_pos_process.start()
        if motorB:
            if self._motorB_pos_process:
                self._motorB_pos_process.shutdown()
                while self._motorB_pos_process.is_alive():
                    pass
            self._motorB_pos_process = MotorProcess(self.motorBWorker,
                                                    motorB)
            self._motorB_pos_process.start()

    def motorAWorker(self, goto_pos):
        """
        Worker function for process gotoPosition
        """
        phy_pos = (self.encoder.getPositions()[0])
        self.motorDirection(motorA=int(goto_pos < phy_pos))
        p_vel = self.p_gain * ((goto_pos[0]*464 + goto_pos[1]) -
                               (phy_pos[0]*464 + phy_pos[1]))
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

    def motorBWorker(self, goto_pos):
        """
        Worker function as a target for the motorB part of gotoPosition
        """
        phy_pos = (self.encoder.getPositions()[1])
        self.motorDirection(motorA=int(goto_pos < phy_pos))
        p_vel = self.p_gain * ((goto_pos[0]*464 + goto_pos[1]) -
                               (phy_pos[0]*464 + phy_pos[1]))
        d_vel = self.d_gain * self.motorB_speed.read()
        if self.motorB_direction.read() == 1:
            d_vel = -d_vel
        vel = p_vel - d_vel

        if vel == abs(vel):
            self.motorDirection(motorB=0)
        else:
            self.motorDirection(motorB=1)
        vel = abs(vel)

        if vel > 1:
            vel = 1

        if self.motorB_speed.read() != vel:
            self.motorSpeed(motorB=vel)


class Position(Structure):
    """
    Setup process-safe position list of tuples
    """
    _fields_ = [('revs', c_long), ('counts', c_long)]


class Encoder(Process):
    """
    Python API for using an arduino as a Quad-encoder
    """
    def __init__(self, serial_ext, clicks_per_rev):
        Process.__init__(self)
        self.revolutions = 0
        self.clicks = 0
        self.clicks_per_rev = clicks_per_rev
        self._posLock = Lock()
        self.ser = serial.Serial(serial_ext)
        for i in range(15):
            self.ser.readline()

        # may need to look this over again on tuesday. but have it here for now
        while not re.match(r"[-]?\d*[,][-]?\d*", self.ser.readline()):
            pass
        self.numMotors = len(self.ser.readline().split(';')[:-1])
        self.positions = Array(Position,
                               [(i, i)for i in range(self.numMotors)])
        self.start()

    def convertPositionStdToTicks(self, position):
        """
        Converts a standard position of form (revs, counts) to
        ticks, just the number of ticks an encoder has logged
        """
        return position[0]*self.clicks_per_rev + position[1]

    def convertPositionTicksToStd(self, position):
        """
        Converts a tick count to standard position of form (revs, counts)
        """
        return (position / self.clicks_per_rev, position % self.clicks_per_rev)

    def run(self):
        while True:
            motors = self.ser.readline()
            motors = motors.split("\n")[0]
            motors = motors.split(';')[:-1]
            temp_positions = list()
            positions_counter = 0
            for motor in motors:
                # regex to filter bad data packets
                if re.match(r"[-]?\d*[,][-]?\d*", motor):
                    self.positions[positions_counter].revs = \
                        int(motor.split(',')[0])
                    self.positions[positions_counter].counts = \
                        int(motor.split(',')[1])
                positions_counter += 1

    def getPositions(self):
        """
        API function so the user doesn't have to
        deal with weird ctype_arrays
        """
        retVal = [(motor.revs, motor.counts) for motor in self.positions]
        return retVal
