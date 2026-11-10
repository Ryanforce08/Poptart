from phoenix6 import CANBus, controls
from phoenix6.configs import TalonFXConfiguration
from phoenix6.hardware import TalonFX
from phoenix6.signals import InvertedValue, NeutralModeValue
from phoenix6.units import rotations_per_second, volt
from wpilib import CANPort, DoubleSolenoid, PneumaticHub


class OctoModule:
    def __init__(
        self, motor_id: int, can_port: CANPort, solenoid_channels: tuple[int, int]
    ) -> None:
        self.motor = TalonFX(motor_id, CANBus(can_port))

        self.config = TalonFXConfiguration()
        self.config.motor_output.neutral_mode = NeutralModeValue.BRAKE
        self.motor.configurator.apply(self.config)

        forward_channel, reverse_channel = solenoid_channels
        self.solenoid = PneumaticHub(can_port).make_double_solenoid(
            forward_channel, reverse_channel
        )

        self._volt_control = controls.VoltageOut(0.0)
        self._velocity_control = controls.VelocityVoltage(0.0)

    def set_inverted(self, inverted: InvertedValue) -> None:
        self.config.motor_output.inverted = inverted
        self.motor.configurator.apply(self.config)

    def set_motor_voltage(self, volts: volt) -> None:
        self.motor.set_control(self._volt_control.with_output(volts))

    def set_motor_velocity(self, speed: rotations_per_second) -> None:
        self.motor.set_control(self._velocity_control.with_velocity(speed))

    def set_solenoid_state(self, state: DoubleSolenoid.Value) -> None:
        self.solenoid.set(state)
