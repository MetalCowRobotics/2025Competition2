package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

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

    // Blue Alliance Positions
    public static final Pose2d[] BLUE_POSITIONS = {
        new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
        new Pose2d(3.2, 4.18, Rotation2d.fromDegrees(-6)),
        new Pose2d(3.7, 2.99, Rotation2d.fromDegrees(55)),
        new Pose2d(5.03, 2.8, Rotation2d.fromDegrees(115)),
        new Pose2d(5.83, 3.87, Rotation2d.fromDegrees(180)),
        new Pose2d(5.3, 5.07, Rotation2d.fromDegrees(-125)),
        new Pose2d(3.9, 5.2, Rotation2d.fromDegrees(-63)),
        new Pose2d(1.166, 1.006, Rotation2d.fromDegrees(0)),
        new Pose2d(1.166, 7.057, Rotation2d.fromDegrees(0))
    };

    // Red Alliance Positions
    public static final Pose2d[] RED_POSITIONS = {
        new Pose2d(14.367, 3.781, Rotation2d.fromDegrees(0)),
        new Pose2d(13.886, 5.012, Rotation2d.fromDegrees(0)),
        new Pose2d(12.621, 5.242, Rotation2d.fromDegrees(0)),
        new Pose2d(11.772, 4.231, Rotation2d.fromDegrees(0)),
        new Pose2d(12.194, 3.077, Rotation2d.fromDegrees(0)),
        new Pose2d(13.526, 2.848, Rotation2d.fromDegrees(0)),
        new Pose2d(16.44, 7.009, Rotation2d.fromDegrees(0)),
        new Pose2d(16.44, 1.010, Rotation2d.fromDegrees(0))
    };

    // Index trackers for cycling through positions
    public static int blueIndex = 0;
    public static int redIndex = 0;

    // Methods to get next position and cycle through arrays
    public static Pose2d getNextBluePosition() {
        if (blueIndex >= BLUE_POSITIONS.length) {
            blueIndex = 0;
        }
        Pose2d position = BLUE_POSITIONS[blueIndex];
        blueIndex = (blueIndex + 1) % BLUE_POSITIONS.length;
        return position;
    }

    public static Pose2d getNextRedPosition() {
        if (redIndex >= RED_POSITIONS.length) {
            redIndex = 0;
        }
        Pose2d position = RED_POSITIONS[redIndex];
        redIndex = (redIndex + 1) % RED_POSITIONS.length;
        return position;
    }

    // Method to reset indices
    public static void resetIndices() {
        blueIndex = 0;
        redIndex = 0;
    }
} 