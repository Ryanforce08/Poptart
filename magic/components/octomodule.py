from magicbot import will_reset_to
from phoenix6 import controls
from phoenix6.configs import TalonFXConfiguration
from phoenix6.hardware import TalonFX
from phoenix6.signals import InvertedValue, NeutralModeValue
from phoenix6.units import rotations_per_second, volt
from wpilib import DoubleSolenoid, PneumaticHub


class OctoModule:
    motor: TalonFX

    pnumatic_hub: PneumaticHub

    solenoid_channels: tuple[int, int]

    motor_control_req = will_reset_to(controls.StaticBrake())

    solenoid_state = will_reset_to(DoubleSolenoid.Value.FORWARD)

    def setup(self) -> None:
        self.config = TalonFXConfiguration()
        self.config.motor_output.neutral_mode = NeutralModeValue.BRAKE
        self.motor.configurator.apply(self.config)

        forward_channel, reverse_channel = self.solenoid_channels

        self.solenoid = self.pnumatic_hub.make_double_solenoid(
            forward_channel, reverse_channel
        )

        self._volt_control = controls.VoltageOut(0.0)
        self._velocity_control = controls.VelocityVoltage(0.0)

    def set_inverted(self, inverted: InvertedValue) -> None:
        self.config.motor_output.inverted = inverted
        self.motor.configurator.apply(self.config)

    def set_motor_voltage(self, volts: volt) -> None:
        self.motor_control_req = self._volt_control.with_output(volts)

    def set_motor_velocity(self, speed: rotations_per_second):
        self.motor_control_req = self._velocity_control.with_velocity(speed)

    def set_solenoid_state(self, state: DoubleSolenoid.Value) -> None:
        self.solenoid_state = state

    def execute(self) -> None:
        self.motor.set_control(self.motor_control_req)

        self.solenoid.set(self.solenoid_state)
