from subsystems.octocanum import Octocanum
from subsystems.octomodule import OctoModule
from wpilib import CANPort, Gamepad, TimedRobot, Timer
from wpimath import apply_deadband, units


class MyRobot(TimedRobot):
    def __init__(self) -> None:
        super().__init__()

        self.can_port = CANPort.CAN_S1

        self.front_left_motor_id = 11
        self.front_left_solenoid_channels = (0, 1)  # Forward, Reverse
        self.front_left = OctoModule(
            self.front_left_motor_id, self.can_port, self.front_left_solenoid_channels
        )

        self.front_right_motor_id = 12
        self.front_right_solenoid_channels = (2, 3)  # Forward, Reverse
        self.front_right = OctoModule(
            self.front_right_motor_id, self.can_port, self.front_right_solenoid_channels
        )

        self.back_left_motor_id = 13
        self.back_left_solenoid_channels = (4, 5)  # Forward, Reverse
        self.back_left = OctoModule(
            self.back_left_motor_id, self.can_port, self.back_left_solenoid_channels
        )

        self.back_right_motor_id = 14
        self.back_right_solenoid_channels = (6, 7)  # Forward, Reverse
        self.back_right = OctoModule(
            self.back_right_motor_id, self.can_port, self.back_right_solenoid_channels
        )

        self.mecanum_gearing = 44.0 / 10.0
        self.differental_gearing = self.mecanum_gearing * 36.0 / 18.0
        self.wheel_radius: units.meters = units.inches_to_meters(2.0)

        self.translationX: units.meters = 0.4
        self.translationY: units.meters = 0.5

        self.octocanum = Octocanum(
            self.front_left,
            self.front_right,
            self.back_left,
            self.back_right,
            self.can_port,
            self.translationX,
            self.translationY,
            self.mecanum_gearing,
            self.differental_gearing,
            self.wheel_radius,
        )

        self.joy = Gamepad(0)

        self.add_periodic(
            self.octocanum.send_networktables, 0.02
        )  # sends data to network tables every 20 milliseconds

    def teleop_periodic(self):
        # only allow output if its over abs(0.1) else 0.0
        left_y = apply_deadband(self.joy.get_left_y(), 0.1)
        left_x = apply_deadband(self.joy.get_left_x(), 0.1)
        right_x = apply_deadband(self.joy.get_right_x(), 0.1)

        if self.joy.get_left_trigger() >= 0.8:  # switch between tank and mecanum
            self.octocanum.arcade_drive(left_x, right_x)
        else:
            self.octocanum.cartesian_drive(left_x, left_y, right_x)

    def autonomous_init(self) -> None:
        self.tank_back_time = 2.0
        self.mec_strafe_time = 1.0 + self.tank_back_time
        self.tank_spin_time = 4.0 + self.mec_strafe_time

        self.auto_start_time = Timer.get_monotonic_timestamp()

    def autonomous_periodic(self) -> None:
        time_in_auto = abs(Timer.get_monotonic_timestamp() - self.auto_start_time)

        if time_in_auto <= self.tank_back_time:
            self.octocanum.arcade_drive(-1.0, 0.0)

        elif time_in_auto <= self.mec_strafe_time:
            self.octocanum.cartesian_drive(0.5, 0.5, 0.5)

        elif time_in_auto <= self.tank_spin_time:
            self.octocanum.arcade_drive(0.0, 1.0)

        else:
            pass
