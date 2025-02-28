package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.constants.AlignmentConstants;

public class LEDDefaultCommand extends Command {
    private final LEDSubsystem ledSubsystem;
    private final CommandSwerveDrivetrain drivetrain;
    private final double proximityThreshold = 1.0; // 1 meter threshold

    public LEDDefaultCommand(LEDSubsystem ledSubsystem, CommandSwerveDrivetrain drivetrain) {
        this.ledSubsystem = ledSubsystem;
        this.drivetrain = drivetrain;
        addRequirements(ledSubsystem);
    }

    @Override
    public void execute() {
        Pose2d currentPose = drivetrain.getState().Pose;
        Pose2d nearestTarget = AlignmentConstants.findClosestTarget(currentPose);
        
        if (nearestTarget != null) {
            double distance = currentPose.getTranslation()
                .getDistance(nearestTarget.getTranslation());
            
            if (distance < proximityThreshold) {
                ledSubsystem.setStrobeGreen();  // Flash green when in range
            } else {
                ledSubsystem.setFixedWhite();  // White when not in range
            }
        } else {
            ledSubsystem.setFixedWhite();  // White when no target found
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
} 