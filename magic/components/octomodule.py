from phoenix6.hardware import TalonFX
from phoenix6.configs import TalonFXConfiguration
from phoenix6.signals import NeutralModeValue, InvertedValue
from phoenix6.units import volt
from magicbot import will_reset_to
from phoenix6 import controls
from wpilib import DoubleSolenoid


class OctoModule:

    motor: TalonFX
    solenoid: DoubleSolenoid

    motor_control_req = will_reset_to(controls.StaticBrake())

    solenoid_state = will_reset_to(DoubleSolenoid.Value.FORWARD)

    def setup(self) -> None:
        self.config = TalonFXConfiguration()
        self.config.motor_output.neutral_mode = NeutralModeValue.BRAKE

        self._volt_control = controls.VoltageOut(0.0)

    def set_inverted(self, inverted: InvertedValue) -> None:
        self.config.motor_output.inverted = inverted

    def set_motor_voltage(self, volts: volt) -> None:
        self.motor_control_req = self._volt_control.with_output(volts)

    def set_solenoid_state(self, state: DoubleSolenoid.Value) -> None:
        self.solenoid_state = state

    def execute(self) -> None:
        self.motor.set_control(self.motor_control_req)

        self.solenoid.set(self.solenoid_state)
