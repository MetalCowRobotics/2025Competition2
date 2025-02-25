package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public final class AlignmentConstants {
    // PID Constants for alignment - Start with conservative values
    public static final double XY_P = 1.0;
    public static final double XY_I = 0.0;
    public static final double XY_D = 0.0;

    public static final double ROTATION_P = 1.0;
    public static final double ROTATION_I = 0.0;
    public static final double ROTATION_D = 0.0;

    // Tolerance for considering alignment complete
    public static final double POSITION_TOLERANCE_METERS = 0.05; // 5 cm
    public static final double ROTATION_TOLERANCE_DEGREES = 2.0; // 2 degrees

    // Left Side Positions
    public static final Pose2d[] LEFT_POSITIONS = {
        new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
        new Pose2d(3.2, 4.18, Rotation2d.fromDegrees(-6)),
        new Pose2d(3.7, 2.99, Rotation2d.fromDegrees(55)),
        new Pose2d(5.03, 2.8, Rotation2d.fromDegrees(115)),
        new Pose2d(5.83, 3.87, Rotation2d.fromDegrees(180)),
        new Pose2d(5.3, 5.07, Rotation2d.fromDegrees(-125)),
        new Pose2d(3.9, 5.2, Rotation2d.fromDegrees(-63)),
        new Pose2d(1.166, 1.006, Rotation2d.fromDegrees(0)),
        new Pose2d(14.367, 3.781, Rotation2d.fromDegrees(0)),
        new Pose2d(13.886, 5.012, Rotation2d.fromDegrees(0)),
        new Pose2d(12.621, 5.242, Rotation2d.fromDegrees(0)),
        new Pose2d(11.772, 4.231, Rotation2d.fromDegrees(0)),
        new Pose2d(12.194, 3.077, Rotation2d.fromDegrees(0)),
        new Pose2d(13.526, 2.848, Rotation2d.fromDegrees(0)),
        new Pose2d(1.166, 7.057, Rotation2d.fromDegrees(0)),
        new Pose2d(16.44, 7.009, Rotation2d.fromDegrees(0)),
        new Pose2d(16.44, 1.010, Rotation2d.fromDegrees(0))
    };

    // Right Side Positions
    public static final Pose2d[] RIGHT_POSITIONS = {
        new Pose2d(3.15, 3.8, Rotation2d.fromDegrees(0)),
        new Pose2d(4.0, 2.8, Rotation2d.fromDegrees(55.5)),
        new Pose2d(5.31, 3.02, Rotation2d.fromDegrees(115)),
        new Pose2d(5.8, 4.21, Rotation2d.fromDegrees(180)),
        new Pose2d(4.96, 5.28, Rotation2d.fromDegrees(-125)),
        new Pose2d(3.62, 5.03, Rotation2d.fromDegrees(-63)),
        new Pose2d(1.214, 1.006, Rotation2d.fromDegrees(225)),
        new Pose2d(14.367, 4.106, Rotation2d.fromDegrees(0)),
        new Pose2d(13.614, 5.188, Rotation2d.fromDegrees(0)),
        new Pose2d(12.346, 5.090, Rotation2d.fromDegrees(0)),
        new Pose2d(11.781, 3.897, Rotation2d.fromDegrees(0)),
        new Pose2d(12.508, 2.899, Rotation2d.fromDegrees(0)),
        new Pose2d(13.809, 2.985, Rotation2d.fromDegrees(0)),
        new Pose2d(1.198, 7.013, Rotation2d.fromDegrees(135)),
        new Pose2d(16.382, 7.009, Rotation2d.fromDegrees(0)),
        new Pose2d(16.470, 1.096, Rotation2d.fromDegrees(0))
    };

    // Method to find closest target
    public static Pose2d findClosestLeftTarget(Pose2d currentPose) {
        Pose2d closest = LEFT_POSITIONS[0];
        double minDistance = Double.MAX_VALUE;
        
        for (Pose2d target : LEFT_POSITIONS) {
            double distance = new Translation2d(currentPose.getX(), currentPose.getY())
                .getDistance(new Translation2d(target.getX(), target.getY()));
            if (distance < minDistance) {
                minDistance = distance;
                closest = target;
            }
        }
        return closest;
    }

    public static Pose2d findClosestRightTarget(Pose2d currentPose) {
        Pose2d closest = RIGHT_POSITIONS[0];
        double minDistance = Double.MAX_VALUE;
        
        for (Pose2d target : RIGHT_POSITIONS) {
            double distance = new Translation2d(currentPose.getX(), currentPose.getY())
                .getDistance(new Translation2d(target.getX(), target.getY()));
            if (distance < minDistance) {
                minDistance = distance;
                closest = target;
            }
        }
        return closest;
    }
} 