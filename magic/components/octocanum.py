import math
from enum import IntEnum

from magicbot import feedback, will_reset_to
from phoenix6.signals import InvertedValue
from wpilib import DoubleSolenoid, PneumaticHub
from wpimath import (
    ChassisVelocities,
    DifferentialDriveKinematics,
    MecanumDriveKinematics,
    Translation2d,
    units,
)

from components.octomodule import OctoModule


class OctoMode(IntEnum):
    DISABLED = 0
    DIFFERENTIAL_DRIVE = 1
    MECANUM_DRIVE = 2


class Octocanum:
    front_left: OctoModule
    front_right: OctoModule
    back_left: OctoModule
    back_right: OctoModule

    pnumatic_hub: PneumaticHub

    translationX: float
    translationY: float

    mecanum_gearing: float
    differental_gearing: float
    wheel_radius: units.meters

    octo_state = will_reset_to(OctoMode.MECANUM_DRIVE)

    speeds = will_reset_to(ChassisVelocities())

    def setup(self) -> None:
        """
        MODULES
        """
        self.modules = [
            self.front_left,
            self.front_right,
            self.back_left,
            self.back_right,
        ]

        for index, module in enumerate(self.modules):
            # Invert motors on right side of drivetrain
            if index % 2 == 0:
                module.set_inverted(InvertedValue.COUNTER_CLOCKWISE_POSITIVE)
            else:
                module.set_inverted(InvertedValue.CLOCKWISE_POSITIVE)

        self.diff_drive = DifferentialDriveKinematics(self.translationX * 2)

        self.mech_drive = MecanumDriveKinematics(
            Translation2d(self.translationX, self.translationY),
            Translation2d(self.translationX, -self.translationY),
            Translation2d(-self.translationX, self.translationY),
            Translation2d(-self.translationX, -self.translationY),
        )

        self.mech_convert = self._meters_to_rotation(
            self.mecanum_gearing, self.wheel_radius
        )  # Convert meters per second into rotations per second
        self.diff_convert = self._meters_to_rotation(
            self.differental_gearing, self.wheel_radius
        )

        """
        PNUMATICS
        """
        self.pnumatic_hub.enable_compressor_hybrid(80, 115)

    """
    INFORMATIONAL METHODS
    """

    def get_mode(self) -> OctoMode:
        return self.octo_state

    @feedback  # sends return to network tables
    def get_octo_mode_nt(self) -> str:
        return self.octo_state.name

    @feedback
    def get_pressure(self):
        self.pnumatic_hub.get_pressure(0)

    @feedback
    def get_pressure_switch(self):
        self.pnumatic_hub.get_pressure_switch()

    """
    CONTROL METHODS
    """

    def arcade_drive(
        self, vx: units.meters_per_second, omega: units.radians_per_second
    ) -> None:
        self.speeds.vx = vx
        self.speeds.omega = omega
        self.octo_state = OctoMode.DIFFERENTIAL_DRIVE

    def cartesian_drive(
        self,
        vx: units.meters_per_second,
        vy: units.meters_per_second,
        omega: units.radians_per_second,
    ) -> None:
        self.speeds.vx = vx
        self.speeds.vy = vy
        self.speeds.omega = omega
        self.octo_state = OctoMode.MECANUM_DRIVE

    """
    EXECUTE
    """

    def execute(self) -> None:
        if self.get_mode() == OctoMode.MECANUM_DRIVE:
            wheel_speeds = self._mech_kinematics(self.speeds)
            convert = self.mech_convert
            solenoid_state = (
                DoubleSolenoid.Value.FORWARD
            )  # Moves piston so mecanum wheel is on ground

        elif self.get_mode() == OctoMode.DIFFERENTIAL_DRIVE:
            wheel_speeds = self._diff_kinematics(self.speeds)
            convert = self.diff_convert
            solenoid_state = (
                DoubleSolenoid.Value.REVERSE
            )  # Moves piston so mecanum wheel is off ground

        else:
            wheel_speeds = [0.0] * 4  # Creates list where all speeds are zero
            convert = lambda _: 0.0  # makes func that returns 0.0
            solenoid_state = DoubleSolenoid.Value.OFF

        for index, module in enumerate(
            self.modules
        ):  # apply everything for every module
            module.set_solenoid_state(solenoid_state)
            module.set_motor_velocity(
                convert(wheel_speeds[index])
            )  # uses correct gearing as it changes for each mode

    """
    INTERNAL FUNCTIONS
    """

    def _mech_kinematics(self, velocities: ChassisVelocities) -> list[float]:
        """
        Returns a iterable velocity list
        """
        speeds = self.mech_drive.to_wheel_velocities(velocities)
        return [
            speeds.front_left,
            speeds.front_right,
            speeds.rear_left,
            speeds.rear_right,
        ]

    def _diff_kinematics(self, velocities: ChassisVelocities) -> list[float]:
        """
        Returns a iterable velocity list
        """
        speeds = self.diff_drive.to_wheel_velocities(velocities)
        return [
            speeds.left,
            speeds.right,
            speeds.left,
            speeds.right,
        ]

    def _meters_to_rotation(self, gearing: float, wheel_radius: units.meters):
        """
        converts meters per second to rotation per second.
        uses anonymous function so it is used like
        ```
        mtp_gear1 = _meters_to_rotation(gear_ratio1, wheel_rad)
        rot_per = mtp_gear1(speed)
        ```
        """

        return lambda speed: speed * wheel_radius * 2 * math.pi / gearing
