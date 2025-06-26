import commands2
import ntcore

from subsystems.intake_subsystem import IntakeSubsystem


class IntakeOut(commands2.Command):

    def __init__(self, intake_sub: IntakeSubsystem) -> None:
        super().__init__()

        self.intake_sub = intake_sub
        self.addRequirements(self.intake_sub)

        self.speed = 0.2
        self.arm_state = "0"

    def execute(self) -> None:
        self.arm_state = ntcore.NetworkTableInstance.getDefault().getTable("arm_table").getStringTopic(
            "arm_state").subscribe("up").get()

        if self.arm_state == "up":
            self.speed = 0.8
        elif self.arm_state == "down":
            self.speed = -0.6

        print(self.arm_state)

        self.intake_sub.set_intake_speed(self.speed)

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        self.intake_sub.set_intake_speed(0)