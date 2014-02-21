import pyfirmata
from pyfirmata import Arduino, util
import serial
import time
from multiprocessing import Process, Lock, Event
from multiprocessing.sharedctypes import Array, Value
from ctypes import Structure, c_long
import re


class MotorProcess(Process):
    """
    Process helper class for motor control allows for graceful exit
    """
    def __init__(self, control_functA, control_functB, locationA, locationB):
        Process.__init__(self)
        self.exit = Event()
        self.control_functA = control_functA
        self.control_functB = control_functB

        self.locationA = Position(locationA[0], locationA[1])
        self.locationB = Position(locationB[0], locationB[1])
    def shutdown(self):
        self.exit.set()

    def run(self):
        while not self.exit.is_set():
            self.control_functA((self.locationA.revs, self.locationA.counts))
            self.control_functB((self.locationB.revs, self.locationB.counts))

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
        self.trigger_funct(self.num_fire)


class ArduControl():
    """
    Python library/API for controlling arduino motorcontrollers
    over USB serial connection.
    """
    def __init__(self, extension, encoder_board=None):
        self.board = pyfirmata.Arduino(extension)

        self._motor_pos_process = None

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
        
        self.control_lock = Lock()

        self.board.servo_config(pin=6,
                                min_pulse=600,
                                max_pulse=2400,
                                angle=35)
        self.board.servo_config(pin=10,
                                min_pulse=600,
                                max_pulse=2400,
                                angle=0)

        self.trigger_time_constant = .5 * .2

        self.p_gain = .014
        self.d_gain = .0052

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
        self.control_lock.acquire()
        self.triggerA.write(0)
        self.control_lock.release()
        time.sleep(num_fire * self.trigger_time_constant * 2)
        self.control_lock.acquire()
        self.triggerA.write(25)
        self.control_lock.release()
        time.sleep(num_fire * self.trigger_time_constant * 2)

    def triggerBWorker(self, num_fire):
        """
        Worker function for fire.  This is specialized for crossfire, and
        should probably be abandoned for production.  But is fine for now.
        """
        self.control_lock.acquire()
        self.triggerB.write(30)
        self.control_lock.release()
        time.sleep(num_fire * self.trigger_time_constant * 3)
        self.control_lock.acquire()
        self.triggerB.write(0)
        self.control_lock.release()
        time.sleep(num_fire * self.trigger_time_constant * 2)
    
    def triggerAOpen(self):
        if self._triggerA_process:
            if self._triggerA_process.is_alive():
                self._triggerA_process.shutdown()
                while self._triggerA_process.is_alive():
                    pass
        self.triggerA.write(0)
    def triggerAClose(self):
        if self._triggerA_process:
            if self._triggerA_process.is_alive():
                self._triggerA_process.shutdown()
                while self._triggerA_process.is_alive():
                    pass
        
        self.triggerA.write(25)
    def triggerBOpen(self):
        if self._triggerB_process:
            if self._triggerB_process.is_alive():
                self._triggerB_process.shutdown()
                while self._triggerB_process.is_alive():
                    pass
        self.triggerB.write(30)
    def triggerBClose(self):
        if self._triggerB_process:
            if self._triggerB_process.is_alive():
                self._triggerB_process.shutdown()
                while self._triggerB_process.is_alive():
                    pass
        

        self.triggerB.write(0)
     
    def brake(self, motorA=None, motorB=None):
        """
        Uses braking functionality given in some arduino motor drivers
        """
        if motorA is not None:
            self.control_lock.acquire()
            self.motorA_brake.write(1)
            self.control_lock.release()

        if motorB is not None:
            self.control_lock.acquire()
            self.motorB_brake.write(1)
            self.control_lock.release()

    def haltMotor(self, motorA=None, motorB=None):
        """
        Stops a given motor, and all processes related to it
        """
        if self._motor_pos_process and self._motor_pos_process.is_alive():
            self._motor_pos_process.shutdown()
            while self._motor_pos_process.is_alive():
                pass
        self.motorSpeed(motorA=1)
        self.motorSpeed(motorA=0)
        self.motorSpeed(motorB=1)
        self.motorSpeed(motorB=0)

    def motorSpeed(self, motorA=None, motorB=None):
        """
        Sets motor to the given speed.  value must be between 0 and 1
        """
        if motorA is not None:
            self.control_lock.acquire()
            self.motorA_speed.write(motorA)
            self.control_lock.release()
        if motorB is not None:
            self.control_lock.acquire()
            self.motorB_speed.write(motorB)
            self.control_lock.release()

    def motorDirection(self, motorA=None, motorB=None):
        """
        sets the direction of the motor to given value
        """
        if motorA is not None:
            self.control_lock.acquire()
            self.motorA_direction.write(motorA)
            self.control_lock.release()

        if motorB is not None:
            self.control_lock.acquire()
            self.motorB_direction.write(motorB)
            self.control_lock.release()

    def motorDirectionToggle(self, motorA=None, motorB=None):
        """
        Switches the direction of the given motor
        """
        if motorA:
            self.control_lock.acquire()
            self.motorA_direction.write(not self.motorA_direction.read())
            self.control_lock.release()
        if motorB:
            self.control_lock.acquire()
            self.motorB_direction.write(not self.motorB_direction.read())
            self.control_lock.release()

    def gotoPosition(self, motorA=None, motorB=None):
        """
        Sends a motor to a given position.
        This takes a tuple of (revs, counts)
        Requires an encoder to function
        """
        
        if motorA is None:
            motorA = self.encoder.getPositions()[0]
        if motorB is None:
            motorB = self.encoder.getPositions()[1]

        if self._motor_pos_process:
            if self._motor_pos_process.is_alive():
                self._motor_pos_process.shutdown()
                while self._motor_pos_process.is_alive():
                    pass
               
            self._motor_pos_process = MotorProcess(self.motorAWorker, self.motorBWorker,
                                                motorA, motorB)
            self._motor_pos_process.start()
        else:
            self._motor_pos_process = MotorProcess(self.motorAWorker, self.motorBWorker,
                                                motorA, motorB)
            self._motor_pos_process.start()


    def motorAWorker(self, goto_pos):
        """
        Worker function for process gotoPosition
        """
        phy_pos = (self.encoder.getPositions()[0])
        self.motorDirection(motorA=int(goto_pos < phy_pos))
        p_vel = self.p_gain * ((goto_pos[0]*464 + goto_pos[1]) -
                               (phy_pos[0]*464 + phy_pos[1]))
        d_vel = self.d_gain * self.encoder.getVelocities()[0]
        self.control_lock.acquire()
        if self.motorA_direction.read() == 1:
            d_vel = -d_vel
        self.control_lock.release()
        vel = p_vel - d_vel
        if vel == abs(vel):
            self.motorDirection(0)
        else:
            self.motorDirection(1)
        vel = abs(vel)
         
        if vel > 1:
            vel = 1
        self.control_lock.acquire() 
        curSpeed = self.motorA_speed.read()
        self.control_lock.release()
        if curSpeed != vel:
            self.motorSpeed(motorA=vel)

    def motorBWorker(self, goto_pos):
        """
        Worker function as a target for the motorB part of gotoPosition
        """
        phy_pos = (self.encoder.getPositions()[1])
        self.motorDirection(motorA=int(goto_pos < phy_pos))
        p_vel = self.p_gain * ((goto_pos[0]*464 + goto_pos[1]) -
                               (phy_pos[0]*464 + phy_pos[1]))
        d_vel = self.d_gain * self.encoder.getVelocities()[1]
        self.control_lock.acquire()
        if self.motorB_direction.read() == 1:
            d_vel = -d_vel
        self.control_lock.release()
        vel = p_vel - d_vel
        

        if vel == abs(vel):
            self.motorDirection(motorB=0)
        else:
            self.motorDirection(motorB=1)
        vel = abs(vel)

        if vel > 1:
            vel = 1
        
        self.control_lock.acquire()
        curVel = self.motorB_speed.read()
        self.control_lock.release()
        if curVel != vel:
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
        self.gameState = Value('i', 1)
        for i in range(15):
            self.ser.readline()

        # may need to look this over again on tuesday. but have it here for now
        while not re.match(r"[-]?\d*[,][-]?\d*", self.ser.readline()):
            pass
        self.numMotors = len(self.ser.readline().split(';')[:-1])
        self.positions = Array(Position,
                               [(i, i)for i in range(self.numMotors)])
        self.velocities = Array('d', 2)
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
        if position == abs(position):
            return (position / self.clicks_per_rev, position % self.clicks_per_rev)
        else:
            return (-(position / -self.clicks_per_rev), position % -self.clicks_per_rev)
    def run(self):
        dt = .001
        start = time.time()
        oldpos = [0, 0]
        while True:
            currentTime = time.time()
            if currentTime - start >= dt:
                tickPos = [self.convertPositionStdToTicks((i.revs, i.counts)) for i in self.positions]
                self.velocities[0] = (abs(oldpos[0] - tickPos[0]) / (currentTime - start))
                self.velocities[1] = (abs(oldpos[1] - tickPos[1]) / (currentTime - start))
                oldpos = [self.convertPositionStdToTicks((i.revs, i.counts)) for i in self.positions]
                start = time.time()
            motors = self.ser.readline()
            motors = motors.split("\n")[0]
            self.gameState.value = int(motors.split(';')[-1])
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
    def getVelocities(self):
        """
        API function so we don't have to deal with c-types
        """
        retVal = [i for i in self.velocities]
        return retVal

    def isGameGoing(self):
        return not self.gameState.value

    def getPositions(self):
        """
        API function so the user doesn't have to
        deal with weird ctype_arrays
        """
        retVal = [(motor.revs, motor.counts) for motor in self.positions]
        return retVal
