from magicbot import MagicRobot
from components.octomodule import OctoModule
from components.octocanum import Octocanum

from phoenix6.hardware import TalonFX
from phoenix6 import CANBus
from wpilib import CANPort, Gamepad
from components.octomodule import OctoModule
from enum import IntEnum


class MyRobot(MagicRobot):

    octocanum: Octocanum

    front_left: OctoModule
    front_right: OctoModule
    back_left: OctoModule
    back_right: OctoModule

    def create_objects(self) -> None:
        self.canbus = CANBus(CANPort.CAN_S1)

        self.front_left_motor = TalonFX(11, self.canbus)
        self.front_right_motor = TalonFX(12, self.canbus)
        self.back_left_motor = TalonFX(13, self.canbus)
        self.back_right_motor = TalonFX(14, self.canbus)

        self.translationX = 0.4
        self.translationY = 0.5

        self.joy = Gamepad(0)

    def teleop_periodic(self):
        if self.joy.get_left_trigger() >= 0.8:
            self.octocanum.arcade_drive(self.joy.get_left_y(), self.joy.get_right_x())
        else:
            self.octocanum.cartesian_drive(
                self.joy.get_left_y(), self.joy.get_left_x(), self.joy.get_right_x()
            )
