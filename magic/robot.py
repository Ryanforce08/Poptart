from components.octocanum import Octocanum
from components.octomodule import OctoModule
from magicbot import MagicRobot
from phoenix6 import CANBus
from phoenix6.hardware import TalonFX
from wpilib import CANPort, Gamepad, PneumaticHub
from wpimath import apply_deadband, units


class MyRobot(MagicRobot):
    octocanum: Octocanum

    front_left: OctoModule
    front_right: OctoModule
    back_left: OctoModule
    back_right: OctoModule

    def create_objects(self) -> None:
        self.can_port = CANPort.CAN_S1
        self.canbus = CANBus(self.can_port)

        self.pnumatic_hub = PneumaticHub(self.can_port)

        self.front_left_motor = TalonFX(11, self.canbus)
        self.front_left_solenoid_channels = (0, 1)  # Forward, Reverse

        self.front_right_motor = TalonFX(12, self.canbus)
        self.front_right_solenoid_channels = (2, 3)  # Forward, Reverse

        self.back_left_motor = TalonFX(13, self.canbus)
        self.back_left_solenoid_channels = (4, 5)  # Forward, Reverse

        self.back_right_motor = TalonFX(14, self.canbus)
        self.back_right_solenoid_channels = (6, 7)  # Forward, Reverse

        self.mecanum_gearing = 44.0 / 10.0
        self.differental_gearing = self.mecanum_gearing * 36.0 / 18.0
        self.wheel_radius = units.inches_to_meters(2.0)

        self.translationX = 0.4
        self.translationY = 0.5

        self.joy = Gamepad(0)

    def teleop_periodic(self):
        left_y = apply_deadband(self.joy.get_left_y(), 0.1)
        left_x = apply_deadband(self.joy.get_left_x(), 0.1)
        right_x = apply_deadband(self.joy.get_right_x(), 0.1)

        if self.joy.get_left_trigger() >= 0.8:  # switch between tank and mecanum
            self.octocanum.arcade_drive(left_x, right_x)
        else:
            self.octocanum.cartesian_drive(left_x, left_y, right_x)
