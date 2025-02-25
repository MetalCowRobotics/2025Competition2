package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

public class Vision extends SubsystemBase {
    private final CommandSwerveDrivetrain drivetrain;
    
    public Vision(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public void periodic() {
        updatePoseEstimates();
    }

    private void updatePoseEstimates() {
        boolean doRejectUpdate = false;

        // Front Left Limelight
        PoseEstimate fleft = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-fleft");
        if (fleft != null) {
            SmartDashboard.putString("Limelight Fleft", fleft.pose.toString());
            SmartDashboard.putNumber("Limelight Fleft.pose.x", fleft.pose.getX());
            SmartDashboard.putNumber("Limelight Fleft.pose.y", fleft.pose.getY());

            doRejectUpdate = shouldRejectUpdate(fleft, 6.0);

            if (!doRejectUpdate) {
                // Set vision measurement standard deviations
                drivetrain.addVisionMeasurement(
                    new Pose2d(
                        -fleft.pose.getX(),
                        fleft.pose.getY(),
                        fleft.pose.getRotation().unaryMinus()
                    ),
                    fleft.timestampSeconds,
                    VecBuilder.fill(0.5, 0.5, 9999999)
                );
            }
        }

        // Front Right Limelight
        doRejectUpdate = false;
        PoseEstimate fright = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-fright");
        if (fright != null) {
            SmartDashboard.putString("Limelight Fright", fright.pose.toString());
            SmartDashboard.putNumber("Limelight Fright.pose.x", fright.pose.getX());
            SmartDashboard.putNumber("Limelight Fright.pose.y", fright.pose.getY());

            doRejectUpdate = shouldRejectUpdate(fright, 0.9);

            if (!doRejectUpdate) {
                drivetrain.addVisionMeasurement(
                    fright.pose,
                    fright.timestampSeconds,
                    VecBuilder.fill(0.5, 0.5, 9999999)
                );
            }
        }
    }

    private boolean shouldRejectUpdate(PoseEstimate estimate, double maxDistance) {
        if (estimate.tagCount == 0) {
            return true;
        }
        
        if (estimate.tagCount == 1 && estimate.rawFiducials.length == 1) {
            if (estimate.rawFiducials[0].ambiguity > 0.3) {
                return true;
            }
            if (estimate.rawFiducials[0].distToCamera > maxDistance) {
                return true;
            }
        }
        
        return false;
    }
} 