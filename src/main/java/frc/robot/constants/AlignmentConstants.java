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

    // Blue Alliance Left Side Positions
    public static final Pose2d[] BLUE_LEFT_POSITIONS = {
        new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
        new Pose2d(3.2, 4.18, Rotation2d.fromDegrees(-6)),
        new Pose2d(5.03, 2.8, Rotation2d.fromDegrees(115)),
        new Pose2d(5.83, 3.87, Rotation2d.fromDegrees(180)),
        new Pose2d(1.166, 7.057, Rotation2d.fromDegrees(0))
    };

    // Blue Alliance Right Side Positions
    public static final Pose2d[] BLUE_RIGHT_POSITIONS = {
        new Pose2d(0, 0.5, Rotation2d.fromDegrees(0)),
        new Pose2d(3.7, 2.99, Rotation2d.fromDegrees(55)),
        new Pose2d(5.3, 5.07, Rotation2d.fromDegrees(-125)),
        new Pose2d(3.9, 5.2, Rotation2d.fromDegrees(-63)),
        new Pose2d(1.166, 1.006, Rotation2d.fromDegrees(0))
    };

    // Red Alliance Left Side Positions
    public static final Pose2d[] RED_LEFT_POSITIONS = {
        new Pose2d(14.367, 3.781, Rotation2d.fromDegrees(0)),
        new Pose2d(12.621, 5.242, Rotation2d.fromDegrees(0)),
        new Pose2d(11.772, 4.231, Rotation2d.fromDegrees(0)),
        new Pose2d(16.44, 7.009, Rotation2d.fromDegrees(0))
    };

    // Red Alliance Right Side Positions
    public static final Pose2d[] RED_RIGHT_POSITIONS = {
        new Pose2d(13.886, 5.012, Rotation2d.fromDegrees(0)),
        new Pose2d(12.194, 3.077, Rotation2d.fromDegrees(0)),
        new Pose2d(13.526, 2.848, Rotation2d.fromDegrees(0)),
        new Pose2d(16.44, 1.010, Rotation2d.fromDegrees(0))
    };

    // Index trackers for cycling through positions
    public static int blueLeftIndex = 0;
    public static int blueRightIndex = 0;
    public static int redLeftIndex = 0;
    public static int redRightIndex = 0;

    // Methods to get next position and cycle through arrays
    public static Pose2d getNextBlueLeftPosition() {
        if (blueLeftIndex >= BLUE_LEFT_POSITIONS.length) {
            blueLeftIndex = 0;
        }
        Pose2d position = BLUE_LEFT_POSITIONS[blueLeftIndex];
        blueLeftIndex = (blueLeftIndex + 1) % BLUE_LEFT_POSITIONS.length;
        return position;
    }

    public static Pose2d getNextBlueRightPosition() {
        if (blueRightIndex >= BLUE_RIGHT_POSITIONS.length) {
            blueRightIndex = 0;
        }
        Pose2d position = BLUE_RIGHT_POSITIONS[blueRightIndex];
        blueRightIndex = (blueRightIndex + 1) % BLUE_RIGHT_POSITIONS.length;
        return position;
    }

    public static Pose2d getNextRedLeftPosition() {
        if (redLeftIndex >= RED_LEFT_POSITIONS.length) {
            redLeftIndex = 0;
        }
        Pose2d position = RED_LEFT_POSITIONS[redLeftIndex];
        redLeftIndex = (redLeftIndex + 1) % RED_LEFT_POSITIONS.length;
        return position;
    }

    public static Pose2d getNextRedRightPosition() {
        if (redRightIndex >= RED_RIGHT_POSITIONS.length) {
            redRightIndex = 0;
        }
        Pose2d position = RED_RIGHT_POSITIONS[redRightIndex];
        redRightIndex = (redRightIndex + 1) % RED_RIGHT_POSITIONS.length;
        return position;
    }

    // Method to reset indices
    public static void resetIndices() {
        blueLeftIndex = 0;
        blueRightIndex = 0;
        redLeftIndex = 0;
        redRightIndex = 0;
    }
} 