package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.constants.AlignmentConstants;

public class LEDAlignmentCommand extends Command {
    private final LEDSubsystem ledSubsystem;
    private final CommandSwerveDrivetrain drivetrain;
    private final double alignmentThreshold = 0.5; // Adjust this value based on your alignment needs (in meters)

    public LEDAlignmentCommand(LEDSubsystem ledSubsystem, CommandSwerveDrivetrain drivetrain) {
        this.ledSubsystem = ledSubsystem;
        this.drivetrain = drivetrain;
        addRequirements(ledSubsystem);
    }

    @Override
    public void initialize() {
        ledSubsystem.setStrobeGreen();
    }

    @Override
    public void execute() {
        // Get the current pose
        Pose2d currentPose = drivetrain.getState().Pose;
        
        // Find the closest target (similar to your AlignToTarget logic)
        Pose2d targetPose = AlignmentConstants.findClosestTarget(currentPose);
        
        if (targetPose != null) {
            // Calculate distance to target
            Transform2d error = targetPose.minus(currentPose);
            double distance = Math.hypot(error.getX(), error.getY());
            
            // If we're close enough to the target, show red strobe
            if (distance < alignmentThreshold) {
                ledSubsystem.setFixedWhite();
            } else {
                ledSubsystem.setStrobeGreen();
            }
        } else {
            ledSubsystem.setStrobeGreen();
        }
    }

    @Override
    public void end(boolean interrupted) {
        ledSubsystem.setStrobeGreen();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
} 