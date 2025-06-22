import ntcore
import rev
import wpilib
from commands2 import SubsystemBase


class IntakeSubsystem(SubsystemBase):
    # Create a new LiftSubsystem

    def __init__(self) -> None:
        super().__init__()

        self.intake_motor = rev.SparkMax(1, rev.SparkMax.MotorType.kBrushless)

    def set_intake_speed(self, speed):
        self.intake_motor.set(speed)
