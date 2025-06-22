import ntcore
import phoenix5
import wpilib
from commands2 import SubsystemBase


class ClimberSubsystem(SubsystemBase):
    # Create a new LiftSubsystem

    def __init__(self) -> None:
        super().__init__()

        self.climber_motor = phoenix5.TalonSRX(3)

    def set_climber_speed(self, speed):
        self.climber_motor.set(phoenix5.ControlMode.PercentOutput, speed)
