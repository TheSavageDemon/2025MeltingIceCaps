from enum import auto, Enum
from typing import Optional

from commands2 import Command, Subsystem, cmd
from ntcore import NetworkTableInstance
from phoenix6 import utils
from wpilib import DriverStation, Mechanism2d, Color8Bit

from subsystems.elevator import ElevatorSubsystem
from subsystems.funnel import FunnelSubsystem
from subsystems.pivot import PivotSubsystem
from subsystems.swerve import SwerveSubsystem
from subsystems.vision import VisionSubsystem


class Superstructure(Subsystem):
    """
    The Superstructure is in charge of handling all subsystems to ensure no conflicts between them.
    """

    class Goal(Enum):
        DEFAULT = auto()
        L4_CORAL = auto()
        L3_CORAL = auto()
        L2_CORAL = auto()
        L1_CORAL = auto()
        L2_ALGAE = auto()
        L3_ALGAE = auto()
        PROCESSOR = auto()
        NET = auto()
        FUNNEL = auto()
        FLOOR = auto()
        CLIMBING = auto()

    # Map each goal to each subsystem state to reduce code complexity
    _goal_to_states: dict[Goal,
            tuple[
                Optional[PivotSubsystem.SubsystemState],
                Optional[ElevatorSubsystem.SubsystemState],
                Optional[FunnelSubsystem.SubsystemState]
            ]] = {
        Goal.DEFAULT: (PivotSubsystem.SubsystemState.STOW, ElevatorSubsystem.SubsystemState.DEFAULT, FunnelSubsystem.SubsystemState.DOWN),
        Goal.L4_CORAL: (PivotSubsystem.SubsystemState.HIGH_SCORING, ElevatorSubsystem.SubsystemState.L4, FunnelSubsystem.SubsystemState.DOWN),
        Goal.L3_CORAL: (PivotSubsystem.SubsystemState.L3_CORAL, ElevatorSubsystem.SubsystemState.L3, FunnelSubsystem.SubsystemState.DOWN),
        Goal.L2_CORAL: (PivotSubsystem.SubsystemState.L2_CORAL, ElevatorSubsystem.SubsystemState.L2, FunnelSubsystem.SubsystemState.DOWN),
        Goal.L1_CORAL: (PivotSubsystem.SubsystemState.LOW_SCORING, ElevatorSubsystem.SubsystemState.L1, FunnelSubsystem.SubsystemState.DOWN),
        Goal.L2_ALGAE: (PivotSubsystem.SubsystemState.ALGAE_INTAKE, ElevatorSubsystem.SubsystemState.L2_ALGAE, FunnelSubsystem.SubsystemState.DOWN),
        Goal.L3_ALGAE: (PivotSubsystem.SubsystemState.ALGAE_INTAKE, ElevatorSubsystem.SubsystemState.L3_ALGAE, FunnelSubsystem.SubsystemState.DOWN),
        Goal.PROCESSOR: (PivotSubsystem.SubsystemState.PROCESSOR_SCORING, ElevatorSubsystem.SubsystemState.DEFAULT, FunnelSubsystem.SubsystemState.DOWN),
        Goal.NET: (PivotSubsystem.SubsystemState.NET_SCORING, ElevatorSubsystem.SubsystemState.NET, FunnelSubsystem.SubsystemState.DOWN),
        Goal.FUNNEL: (PivotSubsystem.SubsystemState.FUNNEL_INTAKE, ElevatorSubsystem.SubsystemState.DEFAULT, FunnelSubsystem.SubsystemState.UP),
        Goal.FLOOR: (PivotSubsystem.SubsystemState.GROUND_INTAKE, ElevatorSubsystem.SubsystemState.DEFAULT, FunnelSubsystem.SubsystemState.DOWN),
        Goal.CLIMBING: (PivotSubsystem.SubsystemState.AVOID_CLIMBER, ElevatorSubsystem.SubsystemState.DEFAULT, FunnelSubsystem.SubsystemState.DOWN)
    }
        
    def __init__(self, drivetrain: SwerveSubsystem, pivot: PivotSubsystem, elevator: ElevatorSubsystem, funnel: FunnelSubsystem, vision: VisionSubsystem) -> None:
        """
        Constructs the superstructure using instance of each subsystem.

        :param drivetrain: Swerve drive base
        :type drivetrain: SwerveSubsystem
        :param pivot: Pivot that rotates our intake
        :type pivot: PivotSubsystem
        :param elevator: Elevator that moves the intake up and down
        :type elevator: ElevatorSubsystem
        :param funnel: Pivot for funnel structure
        :type funnel: FunnelSubsystem
        :param vision: Handles all vision estimates
        :type vision: VisionSubsystem
        """
        super().__init__()
        self.drivetrain = drivetrain
        self.pivot = pivot
        self.elevator = elevator
        self.funnel = funnel
        self.vision = vision

        self._goal = self.Goal.DEFAULT
        self.set_goal_command(self._goal)

        self._elevator_old_state = self.elevator.get_current_state()
        self._pivot_old_state = self.pivot.get_current_state()
        self._pivot_old_setpoint = self.pivot.get_setpoint()

        self._current_goal_pub = NetworkTableInstance.getDefault().getTable("Superstructure").getStringTopic("Current Goal").publish()

        if utils.is_simulation():
            self._superstructure_mechanism = Mechanism2d(1, 5, Color8Bit(0, 0, 105))
            self._superstructure_root = self._superstructure_mechanism.getRoot("Root", 1 / 2, 0.125)
            self._elevator_mech = self._superstructure_root.appendLigament("Elevator", 0.2794, 90, 5, Color8Bit(194, 194, 194))
            self._pivot_mech = self._elevator_mech.appendLigament("Pivot", 0.635, 90, 4, Color8Bit(19, 122, 127))

    def periodic(self):
        if DriverStation.isDisabled():
            return

        pivot_state = self.pivot.get_current_state()
        elevator_state = self.elevator.get_current_state()

        # If the elevator needs to move, check if the elevator has coral or if the pivot could interfere with the elevator. 
        if not self.elevator.is_at_setpoint():
            # Wait for Pivot to leave elevator
            self.pivot.set_desired_state(PivotSubsystem.SubsystemState.AVOID_ELEVATOR)
            self.pivot.freeze()
            if self.pivot.is_in_elevator():
                self.elevator.set_desired_state(ElevatorSubsystem.SubsystemState.IDLE)
                self.elevator.freeze()

        # Unfreeze subsystems if safe
        if not self.pivot.is_in_elevator() and pivot_state is PivotSubsystem.SubsystemState.AVOID_ELEVATOR and elevator_state is ElevatorSubsystem.SubsystemState.IDLE:
            self.elevator.unfreeze()
            self.elevator.set_desired_state(self._elevator_old_state)

        if self.elevator.is_at_setpoint() and pivot_state is PivotSubsystem.SubsystemState.AVOID_ELEVATOR:
            self.pivot.unfreeze()
            self.pivot.set_desired_state(self._pivot_old_state)

        # Update old states only when necessary
        if pivot_state is not PivotSubsystem.SubsystemState.AVOID_ELEVATOR:
            self._pivot_old_state = pivot_state
            self._pivot_old_setpoint = self.pivot.get_setpoint()

        if elevator_state is not ElevatorSubsystem.SubsystemState.IDLE:
            self._elevator_old_state = elevator_state

    def simulationPeriodic(self) -> None:
        self._elevator_mech.setLength(self.elevator.get_height())
        self._pivot_mech.setAngle(self.pivot.get_position() * 360 - 90)

    def _set_goal(self, goal: Goal) -> None:
        self._goal = goal

        pivot_state, elevator_state, funnel_state = self._goal_to_states.get(goal, (None, None, None))
        if pivot_state:
            self.pivot.set_desired_state(pivot_state)
        if elevator_state:
            self.elevator.set_desired_state(elevator_state)
        if funnel_state:
            self.funnel.set_desired_state(funnel_state)

        self._current_goal_pub.set(goal.name)

    def set_goal_command(self, goal: Goal) -> Command:
        """
        Return a command that sets the superstructure goal to whatever the desired goal is.

        :param goal: The desired goal
        :type goal:  Goal
        :return:     A command that will set the desired goal
        :rtype:      Command
        """
        return cmd.runOnce(lambda: self._set_goal(goal), self)
