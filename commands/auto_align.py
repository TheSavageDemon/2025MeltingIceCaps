# from commands2 import Command
# from phoenix6.swerve.requests import FieldCentric, FieldCentricFacingAngle
# from phoenix6.swerve.swerve_drivetrain import DriveMotorT, SteerMotorT, EncoderT, SwerveControlParameters
# from subsystems.swerve import DriverAssist
# from wpilib import DriverStation, Notifier, RobotController, Field2d, SmartDashboard
# from wpimath.geometry import Rotation2d, Pose2d, Translation2d



# class AutoAlign(Command):

#     def __init__(self, driver_assist: DriverAssist):
#         super().__init__()
#         self.driver_assist = driver_assist
#         self.addRequirements(driver_assist)
#         self._field_centric_facing_angle = FieldCentricFacingAngle()

#     def execute(self):

#         current_pose = parameters.current_pose

#         if DriverStation.getAlliance() == DriverStation.Alliance.kBlue:
            
#             if self.branch_side == DriverAssist.BranchSide.LEFT:
#                 self._target_pose = self.findClosestPose(current_pose, self._blue_branch_left_targets) 
#             else:
#                 self._target_pose = self.findClosestPose(current_pose, self._blue_branch_right_targets)

#         elif DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            
#             if self.branch_side == DriverAssist.BranchSide.LEFT:
#                 self._target_pose = self.findClosestPose(current_pose, self._red_branch_left_targets)
#             else:
#                 self._target_pose = self.findClosestPose(current_pose, self._red_branch_right_targets)


#         target_direction = self._target_pose.rotation()
#         rotated_coordinate = Translation2d(self.velocity_x, self.velocity_y).rotateBy(-target_direction)
#         velocity_towards_pose = rotated_coordinate.X() * self.max_speed
#         field_relative_velocity = Translation2d(velocity_towards_pose, horizontal_velocity).rotateBy(target_direction)

#     def isFinished(self):
#         return False