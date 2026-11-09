from phoenix6.hardware import TalonFX
from phoenix6.configs import TalonFXConfiguration
from phoenix6.signals import NeutralModeValue, InvertedValue
from phoenix6.units import volt
from magicbot import will_reset_to
from phoenix6 import controls
from wpilib import DoubleSolenoid, MecanumDrive
from wpimath import (
    DifferentialDriveKinematics,
    MecanumDriveKinematics,
    Translation2d,
    ChassisVelocities,
    MecanumDriveWheelVelocities,
)
from components.octomodule import OctoModule
from enum import IntEnum


class OctoMode(IntEnum):
    DISABLED = 0
    DIFFERENTIAL_DRIVE = 1
    MECHANUM_DRIVE = 2


class Octocanum:

    front_left: OctoModule
    front_right: OctoModule
    back_left: OctoModule
    back_right: OctoModule

    translationX: float
    translationY: float

    octo_state = will_reset_to(OctoMode.MECHANUM_DRIVE)

    speeds = will_reset_to(ChassisVelocities())

    def setup(self) -> None:
        self.modules = [
            self.front_left,
            self.front_right,
            self.back_left,
            self.back_right,
        ]

        self.diff_drive = DifferentialDriveKinematics(self.translationX * 2)
        self.mech_drive = MecanumDriveKinematics(
            Translation2d(self.translationX, self.translationY),
            Translation2d(self.translationX, -self.translationY),
            Translation2d(-self.translationX, self.translationY),
            Translation2d(-self.translationX, -self.translationY),
        )

    def get_mode(self):
        return self.octo_state

    def arcade_drive(self, vx, omega):
        self.speeds.vx = vx
        self.speeds.omega = omega
        self.octo_state = OctoMode.DIFFERENTIAL_DRIVE

    def cartesian_drive(self, vx, vy, omega):
        self.speeds.vx = vx
        self.speeds.vy = vy
        self.speeds.omega = omega
        self.octo_state = OctoMode.MECHANUM_DRIVE

    def set_wheel_speeds(self, front_left, front_right, back_left, back_right):
        self.modules[0].set_motor_voltage(front_left)
        self.modules[1].set_motor_voltage(front_right)
        self.modules[2].set_motor_voltage(back_left)
        self.modules[3].set_motor_voltage(back_right)

    def execute(self):
        if self.get_mode() == OctoMode.MECHANUM_DRIVE:
            wheel_speeds = self.mech_drive.to_wheel_velocities(
                self.speeds, Translation2d()
            )
            self.set_wheel_speeds(
                wheel_speeds.front_left,
                wheel_speeds.front_right,
                wheel_speeds.rear_left,
                wheel_speeds.rear_right,
            )
            for i in self.modules:
                i.set_solenoid_state(DoubleSolenoid.Value.FORWARD)
        elif self.get_mode() == OctoMode.DIFFERENTIAL_DRIVE:
            wheel_speeds = self.diff_drive.to_wheel_velocities(self.speeds)
            self.set_wheel_speeds(
                wheel_speeds.left,
                wheel_speeds.right,
                wheel_speeds.left,
                wheel_speeds.right,
            )
            for i in self.modules:
                i.set_solenoid_state(DoubleSolenoid.Value.REVERSE)
        else:
            for i in self.modules:
                i.set_solenoid_state(DoubleSolenoid.Value.OFF)
