import ntcore
import rev
import wpilib
from commands2 import SubsystemBase
from rev import ClosedLoopConfig


class SwingArmSubsystem(SubsystemBase):
    # Create a new LiftSubsystem

    def __init__(self) -> None:
        super().__init__()

        self.arm_motor = rev.SparkMax(2, rev.SparkMax.MotorType.kBrushless)

        self.arm_state = "0"

        # define the wrist subsystem's network table
        nt_instance = ntcore.NetworkTableInstance.getDefault()
        arm_table = nt_instance.getTable("arm_table")

        self.arm_position_entry = arm_table.getDoubleTopic("arm_position").publish()
        self.arm_pid_output_entry = arm_table.getDoubleTopic("arm_pid_output").publish()
        self.arm_state_entry = arm_table.getStringTopic("arm_state").publish()

    def periodic(self):
        self.arm_position_entry.set(self.get_swing_arm_position())


        if self.get_swing_arm_position() < 0.595:
            self.arm_state = "up"
        elif self.get_swing_arm_position() > 0.595:
            self.arm_state = "down"

        self.arm_state_entry.set(self.arm_state)

    def set_swing_arm_speed(self, speed):
        self.arm_motor.set(speed)

    def get_swing_arm_position(self):
        return self.arm_motor.getAbsoluteEncoder().getPosition()