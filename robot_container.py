import math

import commands2
import ntcore
import wpilib
import libgrapplefrc

from commands2 import cmd, WaitCommand
from commands2.cmd import waitSeconds
from wpimath.controller import PIDController, ProfiledPIDControllerRadians, HolonomicDriveController
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator

from commands.climber_down import ClimberDown
from commands.climber_up import ClimberUp
from commands.input_drive import InputDrive
from constants.position_constants import PositionConstants
from commands.drive_command import DriveCommand
from commands.intake_out import IntakeOut
from commands.swing_arm_down import SwingArmDown
from commands.swing_arm_to_position import SwingArmToPosition
from commands.swing_arm_up import SwingArmUp
from constants.swerve_constants import OIConstants, AutoConstants, DriveConstants
from subsystems.drive_subsystem import DriveSubsystem
from subsystems.vision_subsystem import VisionSubsystem
from subsystems.swing_arm_subsystem import SwingArmSubsystem
from subsystems.intake_subsystem import IntakeSubsystem
from subsystems.climber_subsystem import ClimberSubsystem


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        # The robot's subsystems
        self.drive_subsystem = DriveSubsystem()
        self.vision_subsystem = VisionSubsystem()
        self.swing_arm_subsystem = SwingArmSubsystem()
        self.intake_subsystem = IntakeSubsystem()
        self.climber_subsystem = ClimberSubsystem()

        # The driver's controller
        self.driver_controller = wpilib.Joystick(OIConstants.kDriverControllerPort)
        self.operator_controller = wpilib.Joystick(OIConstants.kOperatorControllerPort)

        # Configure the button bindings
        self.configure_button_bindings()

        # Configure default commands
        self.drive_subsystem.setDefaultCommand(
            DriveCommand(self.drive_subsystem)
        )
        # Configure Auto Chooser
        self.chooser = wpilib.SendableChooser()
        self.do_nothing = "Do Nothing"
        self.drive_forward = "Drive Forward"
        self.mid_one_coral = "Middle One Coral"

        self.chooser.setDefaultOption("Shoot 1 Only", self.do_nothing)
        self.chooser.addOption("Drive Forward", self.drive_forward)
        self.chooser.addOption("Middle One Coral", self.mid_one_coral)


    def configure_button_bindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """
        #commands2.button.JoystickButton(self.operator_controller, 1).whileTrue(
        #    SwingArmToPosition(self.swing_arm_subsystem,0.4)
        #)

    def disable_pid_subsystems(self) -> None:
        """Disables all ProfiledPIDSubsystem and PIDSubsystem instances.
        This should be called on robot disable to prevent integral windup."""

    def get_autonomous_command(self) -> commands2.command:
        """Use this to pass the autonomous command to the main {@link Robot} class.

        )
        commands2.button.JoystickButton(self.operator_controller, 8).onTrue(
            SwingArmToPosition(self.swing_arm_subsystem, PositionConstants.kCoralThreeSwingArm)
        )
        # Level 4 Coral Deposit
        commands2.button.JoystickButton(self.operator_controller, 7).onTrue(
            LiftToPosition(self.lift_subsystem, PositionConstants.kCoralFourLift)
        )
        commands2.button.JoystickButton(self.operator_controller, 7).onTrue(
            WristToPosition(self.wrist_subsystem, PositionConstants.kCoralFourWrist)
        )
        commands2.button.JoystickButton(self.operator_controller, 7).onTrue(
            SwingArmToPosition(self.swing_arm_subsystem, PositionConstants.kCoralFourSwingArm)
        )
        # Ground Algae
        commands2.button.JoystickButton(self.operator_controller, 1).onTrue(
        :returns: the command to run in autonomous
        """
        # Create config for trajectory
        config = TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared,
        )
        # Add kinematics to ensure max speed is actually obeyed
        config.setKinematics(DriveConstants.kDriveKinematics)

        # Forward Auto Trajectory to follow. All units in meters.
        forward_trajectory = TrajectoryGenerator.generateTrajectory(
            # Start at the origin facing the +X direction
            Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            # Pass through these two interior waypoints, making an 's' curve path
            [],
            # End 3 meters straight ahead of where we started, facing forward
            Pose2d(0, -3, Rotation2d.fromDegrees(0)),
            config,
        )

        thetaController = ProfiledPIDControllerRadians(
            AutoConstants.kPThetaController,
            0,
            0,
            AutoConstants.kThetaControllerConstraints,
        )
        thetaController.enableContinuousInput(-math.pi, math.pi)

        holonomic_controller = HolonomicDriveController(PIDController(AutoConstants.kPXController, 0, 0),
                                                        PIDController(AutoConstants.kPYController, 0, 0),
                                                        thetaController)

        # forward_trajectory_command = commands2.SwerveControllerCommand(
        #     forward_trajectory,
        #     self.drive_subsystem.get_pose,  # Functional interface to feed supplier
        #     DriveConstants.kDriveKinematics,
        #     # Position controllers
        #     holonomic_controller,
        #     self.drive_subsystem.set_module_states(),
        #     (self.drive_subsystem,),
        # )

        # Start Auto Logic
        return commands2.ParallelDeadlineGroup(
            WaitCommand(1),
            InputDrive(self.drive_subsystem, 0.4, 0, 0)
        )