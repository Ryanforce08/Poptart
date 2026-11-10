import math
from enum import IntEnum

import telemetry
from phoenix6.signals import InvertedValue
from wpilib import CANPort, DoubleSolenoid, PneumaticHub
from wpimath import (
    ChassisVelocities,
    DifferentialDriveKinematics,
    MecanumDriveKinematics,
    Translation2d,
    units,
)

from subsystems.octomodule import OctoModule


class OctoMode(IntEnum):
    DISABLED = 0
    DIFFERENTIAL_DRIVE = 1
    MECANUM_DRIVE = 2


class Octocanum:
    octo_state = OctoMode.MECANUM_DRIVE

    speeds = ChassisVelocities()

    def __init__(
        self,
        front_left: OctoModule,
        front_right: OctoModule,
        back_left: OctoModule,
        back_right: OctoModule,
        can_port: CANPort,
        translationX: units.meters,
        translationY: units.meters,
        mecanum_gearing: float,
        differental_gearing: float,
        wheel_radius: units.meters,
    ) -> None:
        """
        MODULES
        """
        self.modules = [
            front_left,
            front_right,
            back_left,
            back_right,
        ]

        for index, module in enumerate(self.modules):
            # Invert motors on right side of drivetrain
            if index % 2 == 0:
                module.set_inverted(InvertedValue.COUNTER_CLOCKWISE_POSITIVE)
            else:
                module.set_inverted(InvertedValue.CLOCKWISE_POSITIVE)

        self.diff_drive = DifferentialDriveKinematics(translationX * 2)

        self.mech_drive = MecanumDriveKinematics(
            Translation2d(translationX, translationY),
            Translation2d(translationX, -translationY),
            Translation2d(-translationX, translationY),
            Translation2d(-translationX, -translationY),
        )

        self.mech_convert = self._meters_to_rotation(
            mecanum_gearing, wheel_radius
        )  # Convert meters per second into rotations per second
        self.diff_convert = self._meters_to_rotation(differental_gearing, wheel_radius)

        """
        PNUMATICS
        """
        self.pnumatic_hub = PneumaticHub(can_port)
        self.pnumatic_hub.enable_compressor_hybrid(80, 115)

    """
    INFORMATIONAL METHODS
    """

    def get_octo_mode_nt(self) -> str:
        return self.octo_state.name

    def get_pressure(self):
        self.pnumatic_hub.get_pressure(0)

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

        wheel_speeds = self._diff_kinematics(self.speeds)

        for index, module in enumerate(
            self.modules
        ):  # apply everything for every module
            module.set_solenoid_state(DoubleSolenoid.Value.REVERSE)
            module.set_motor_velocity(
                self.diff_convert(wheel_speeds[index])
            )  # uses correct gearing as it changes for each mode

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

        wheel_speeds = self._mech_kinematics(self.speeds)

        for index, module in enumerate(
            self.modules
        ):  # apply everything for every module
            module.set_solenoid_state(DoubleSolenoid.Value.REVERSE)
            module.set_motor_velocity(
                self.mech_convert(wheel_speeds[index])
            )  # uses correct gearing as it changes for each mode

    def send_networktables(self) -> None:
        table = telemetry.get_table("octocanum")
        table.log("octo_mode", self.get_octo_mode_nt())
        table.log("pressure", self.get_pressure())
        table.log("pressure_switch", self.get_pressure_switch())

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
