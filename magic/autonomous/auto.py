from components.octocanum_state import OctocanumState
from magicbot import AutonomousStateMachine, timed_state


class TestAuto(AutonomousStateMachine):
    MODE_NAME = "Test Auto"
    DEFAULT = True

    octocanum_state: OctocanumState

    @timed_state(
        duration=2.0, next_state="mec_strafe", first=True
    )  # runs this state for 2 sec then goes to "mec_strafe" state
    def tank_back(self):
        self.octocanum_state.arcade_drive(-1.0, 0.0)  # Drive backwards in tank drive

    @timed_state(
        duration=1.0, next_state="tank_spin"
    )  # runs this state for 1 sec then goes to "tank_spin" state
    def mec_strafe(self):
        self.octocanum_state.cartesian_drive(0.5, 0.5, 0.5)

    @timed_state(duration=4)  # runs this state for 4 sec then ends auto routine
    def tank_spin(self):
        self.octocanum_state.arcade_drive(0.0, 1.0)
