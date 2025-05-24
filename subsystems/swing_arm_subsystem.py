import ntcore
import rev
import wpilib
from commands2 import SubsystemBase


class ArmSubsystem(SubsystemBase):
    # Create a new LiftSubsystem

    def __init__(self) -> None:
        super().__init__()

        self.arm_motor = rev.SparkFlex(1, rev.SparkMax.MotorType.kBrushless)

        # define the wrist subsystem's network table
        nt_instance = ntcore.NetworkTableInstance.getDefault()
        arm_table = nt_instance.getTable("arm_table")

        self.arm_position_entry = arm_table.getDoubleTopic("arm_position").publish()
        self.arm_pid_output_entry = arm_table.getDoubleTopic("arm_pid_output").publish()

    def periodic(self):
        self.arm_position_entry.set(self.get_arm_position())

    def set_arm_speed(self, speed):
        self.arm_motor.set(speed)

    def get_arm_position(self):
        return self.arm_motor.getEncoder().getPosition()