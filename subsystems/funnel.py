from enum import auto, Enum

from phoenix6.configs import CurrentLimitsConfigs, TalonFXConfiguration, MotionMagicConfigs
from phoenix6.controls import VoltageOut, MotionMagicVoltage
from phoenix6.hardware import TalonFX
from phoenix6.signals import InvertedValue, NeutralModeValue
from wpimath.geometry import Pose3d, Rotation3d, Translation3d
from wpimath.system.plant import DCMotor
from wpimath.units import rotationsToRadians

from constants import Constants
from subsystems import StateSubsystem


class FunnelSubsystem(StateSubsystem):
    """
    The FunnelSubsystem is responsible for handling the funnel mechanism,
    ensuring it stays out of the way during climbing and for accurate positioning for funnel intaking.
    """

    class SubsystemState(Enum):
        UP = auto()
        DOWN = auto()

    _state_configs: dict[SubsystemState, float] = {
        SubsystemState.UP: Constants.FunnelConstants.CORAL_STATION_POSITION,
        SubsystemState.DOWN: Constants.FunnelConstants.STOWED_POSITION,
    }

    _funnel_config = TalonFXConfiguration()
    (_funnel_config.feedback
     .with_sensor_to_mechanism_ratio(Constants.FunnelConstants.GEAR_RATIO)
    )
    _funnel_config.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE
    _funnel_config.motor_output.neutral_mode = NeutralModeValue.BRAKE
    _funnel_config.with_slot0(Constants.FunnelConstants.GAINS)
    _funnel_config.with_motion_magic(MotionMagicConfigs().with_motion_magic_cruise_velocity(Constants.FunnelConstants.CRUISE_VELOCITY).with_motion_magic_acceleration(Constants.FunnelConstants.MM_ACCELERATION))
    _funnel_config.with_current_limits(CurrentLimitsConfigs()
                                       .with_supply_current_limit_enable(True)
                                       .with_supply_current_limit(Constants.FunnelConstants.SUPPLY_CURRENT)
                                       .with_supply_current_lower_time(0)
                                       .with_stator_current_limit_enable(True)
                                       .with_stator_current_limit(Constants.FunnelConstants.STATOR_CURRENT)
                                       )

    def __init__(self) -> None:
        super().__init__("Funnel", self.SubsystemState.DOWN)

        self._funnel_motor = TalonFX(Constants.CanIDs.FUNNEL_TALON)

        self._funnel_motor.configurator.apply(self._funnel_config)

        self._funnel_motor.set_position(0)

        self._add_talon_sim_model(self._funnel_motor, DCMotor.falcon500FOC(), Constants.FunnelConstants.GEAR_RATIO)

        self._position_request = MotionMagicVoltage(0)
        self._brake_request = VoltageOut(0)

    def set_desired_state(self, desired_state: SubsystemState) -> None:
        if not super().set_desired_state(desired_state):
            return

        position = self._state_configs.get(desired_state, Constants.FunnelConstants.STOWED_POSITION)
        self._position_request.position = position
        self._funnel_motor.set_control(self._position_request)

    def get_component_pose(self) -> Pose3d:
        return Pose3d(Translation3d(-0.311150, 0, 0.703243), Rotation3d(0, -rotationsToRadians(self._funnel_motor.get_position().value), 0))

    def get_target_pose(self) -> Pose3d:
        """Works the same as self.get_component_pose, but instead with the PIDReference rather than current position."""
        return Pose3d(Translation3d(-0.311150, 0, 0.703243), Rotation3d(0, -rotationsToRadians(self._funnel_motor.get_closed_loop_reference().value), 0))
