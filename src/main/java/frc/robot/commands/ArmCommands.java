package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Intake;

public final class ArmCommands {
    private final Elevator elevator;
    private final Wrist wrist;
    private final Intake intake;

    public ArmCommands(Elevator elevator, Wrist wrist, Intake intake) {
        this.elevator = elevator;
        this.wrist = wrist;
        this.intake = intake;
    }

    // Rest Position Command
    public Command goToRest() {
        return elevator.goToRestCommand().alongWith(wrist.goToRestCommand()).andThen(intake.stopIntakeCommand());
    }

    // Source Position with Intake Command
    public Command goToSource() {
        return elevator.goToSourceCommand().alongWith(wrist.goToSourceCommand()).andThen(intake.startIntakeCommand());
    }

    // L2 Position Command
    public Command goToL2() {
        return elevator.goToL2Command().alongWith(wrist.goToL3Command()).andThen(intake.stallIntakeCommand());
    }

    // L3 Position Command
    public Command goToL3() {
        return elevator.goToL3Command().alongWith(wrist.goToL3Command()).andThen(intake.stallIntakeCommand());
    }

    // L4 Position Command
    public Command goToL4() {
        return elevator.goToL4Command().alongWith(wrist.goToL4Command()).andThen(intake.stallIntakeCommand());
    }

    // Alage Position Commands
    public Command goToAlgaeL3() {
        return elevator.goToAlgaeL3Command().alongWith(wrist.goToAlage()).andThen(intake.slowReverseIntakeCommand());
    }

    public Command goToAlgaeL2() {
        return elevator.goToAlgaeL2Command().alongWith(wrist.goToAlage()).andThen(intake.slowReverseIntakeCommand());
    }

    // Stop All Command
    public Command stopAll() {
        return intake.stopIntakeCommand();
    }
} 