# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

"""
The constants module is a convenience place for teams to hold robot-wide
numerical or boolean constants. Don't use this for any other purpose!
"""

import math

from wpimath import units
from wpimath.geometry import Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.trajectory import TrapezoidProfileRadians

from rev import SparkMax, SparkFlex, SparkBaseConfig


class NeoMotorConstants:
    kFreeSpeedRpm = 5676


class VortexMotorConstants:
    kFreeSpeedRpm = 6784


class DriveConstants:
    # Driving Parameters - Note that these are not the maximum capable speeds of
    # the robot, rather the allowed maximum speeds
    kMaxSpeedMetersPerSecond = 4
    kMaxAngularSpeed = math.tau  # radians per second

    kDirectionSlewRate = 1.2  # radians per second
    kMagnitudeSlewRate = 1.8  # percent per second (1 = 100%)
    kRotationalSlewRate = 2.0  # percent per second (1 = 100%)

    # Chassis configuration
    kTrackWidth = units.inchesToMeters(24.25)
    # Distance between centers of right and left wheels on robot
    kWheelBase = units.inchesToMeters(27.875)

    # Distance between front and back wheels on robot
    kModulePositions = [
        Translation2d(kWheelBase / 2, kTrackWidth / 2),
        Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
    ]
    kDriveKinematics = SwerveDrive4Kinematics(*kModulePositions)

    # Angular offsets of the modules relative to the chassis in radians
    kFrontLeftChassisAngularOffset = -math.pi / 2
    kFrontRightChassisAngularOffset = 0
    kBackLeftChassisAngularOffset = math.pi
    kBackRightChassisAngularOffset = math.pi / 2

    # SPARK MAX CAN IDs
    kFrontLeftDrivingCanId = 18 #12
    kRearLeftDrivingCanId = 16 #16
    kFrontRightDrivingCanId = 14 #14
    kRearRightDrivingCanId = 12 #18

    kFrontLeftTurningCanId = 17 #11
    kRearLeftTurningCanId = 15 #15
    kFrontRightTurningCanId = 13 #13
    kRearRightTurningCanId = 11 #17

    kGyroReversed = False


class ModuleConstants:
    # The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    # This changes the drive speed of the module (a pinion gear with more teeth will result in a
    # robot that drives faster).
    kDrivingMotorPinionTeeth = 14

    # Invert the turning encoder, since the output shaft rotates in the opposite direction of
    # the steering motor in the MAXSwerve Module.
    kTurningEncoderInverted = True

    # Calculations required for driving motor conversion factors and feed forward
    kDrivingMotorFreeSpeedRps = VortexMotorConstants.kFreeSpeedRpm / 60
    kWheelDiameterMeters = 0.0762
    kWheelCircumferenceMeters = kWheelDiameterMeters * math.pi
    # 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15)
    kDriveWheelFreeSpeedRps = (
                                      kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters
                              ) / kDrivingMotorReduction

    kDrivingEncoderPositionFactor = (
                                            kWheelDiameterMeters * math.pi
                                    ) / kDrivingMotorReduction  # meters
    kDrivingEncoderVelocityFactor = (
                                            (kWheelDiameterMeters * math.pi) / kDrivingMotorReduction
                                    ) / 60.0  # meters per second

    kTurningEncoderPositionFactor = math.tau  # radian
    kTurningEncoderVelocityFactor = math.tau / 60.0  # radians per second

    kTurningEncoderPositionPIDMinInput = 0  # radian
    kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor  # radian

    kDrivingP = 0.04
    kDrivingI = 0
    kDrivingD = 0
    kDrivingFF = 1 / kDriveWheelFreeSpeedRps
    kDrivingMinOutput = -1
    kDrivingMaxOutput = 1

    kTurningP = 1
    kTurningI = 0
    kTurningD = 0
    kTurningFF = 0
    kTurningMinOutput = -1
    kTurningMaxOutput = 1

    kDrivingMotorIdleMode = SparkBaseConfig.IdleMode.kBrake
    kTurningMotorIdleMode = SparkBaseConfig.IdleMode.kBrake

    kDrivingMotorCurrentLimit = 60  # amp
    kTurningMotorCurrentLimit = 20  # amp


class OIConstants:
    kDriverControllerPort = 0
    kDriveDeadband = 0.05

    kOperatorControllerPort = 2


class AutoConstants:
    kMaxSpeedMetersPerSecond = 3
    kMaxAccelerationMetersPerSecondSquared = 3
    kMaxAngularSpeedRadiansPerSecond = math.pi
    kMaxAngularSpeedRadiansPerSecondSquared = math.pi

    kPXController = 1
    kPYController = 1
    kPThetaController = 1

    # Constraint for the motion profiled robot angle controller
    kThetaControllerConstraints = TrapezoidProfileRadians.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared
    )
