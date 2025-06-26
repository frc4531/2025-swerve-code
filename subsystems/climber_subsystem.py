import ntcore
from phoenix6.hardware import TalonFX
import wpilib
from commands2 import SubsystemBase


class ClimberSubsystem(SubsystemBase):
    # Create a new LiftSubsystem

    def __init__(self) -> None:
        super().__init__()

        self.climber_motor = TalonFX(3, "rio")

    def set_climber_speed(self, speed):
        self.climber_motor.setVoltage(speed)
