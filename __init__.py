import pyfirmata
import serial
from pyfirmata import Arduino, util

class ArduControl

    def __init__(self, extension, board=None):
        if board is not None:
            self.board = board
        else:
            self.board = pyFirmata.Arduino(extension)

        self.motorA_speed = self.board.get_pin("d:3:p")
        self.motorA_direction = self.board.get_pin("d:12:o")

        self.motorB_speed = self.board.get_pin("d:11:p")
        self.motorB_direction = self.board.get_pin("d:13:0")

        self.ser = serial.Serial('dev/acm1')

    def motorSpeed(motorA, motorB):
        if motorA is not None:
            self.motorA_speed.write(motorA)

        if motorB is not None:
            self.motorB_speed.write(motorB)

    def motorDirection(motorA, motorB):
        if motorA is not None:
            self.motorA_direction.write(motorA)

        if motorB is not None:
            self.motorB_direction.write(motorB)

    def motorDirectionToggle(motorA, motorB):
        if motorA:

