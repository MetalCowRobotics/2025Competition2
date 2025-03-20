package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public final class AlignmentConstants {
    // PID Constants for alignment - Increased for faster response
    public static final double XY_P = 3.0;
    public static final double XY_I = 0.01;
    public static final double XY_D = 0.1; 

    public static final double ROTATION_P = 3.0; 
    public static final double ROTATION_I = 0.01;
    public static final double ROTATION_D = 0.1; 

    // Tolerance for considering alignment complete
    public static final double POSITION_TOLERANCE_METERS = 0.015;
    public static final double ROTATION_TOLERANCE_DEGREES = 1.0;

    // Offsets for alignment |Tag 7 Pose - Recorded Pose|
    public static final double TAG_LEFT_OFFSET_X = 14.344-13.8927;
    public static final double TAG_LEFT_OFFSET_Y = 4.0260-3.867;
    public static final double TAG_RIGHT_OFFSET_X = 14.358-13.8927;
    public static final double TAG_RIGHT_OFFSET_Y = 4.146-3.867;

    // Holds All Target Positions
    public static final Pose2d[] LEFT_POSITIONS = calculateLeftTargetsWithOffset();
    public static final Pose2d[] RIGHT_POSITIONS = calculateRightTargetsWithOffset();

    private static Pose2d[] calculateRightTargetsWithOffset() {
        Pose2d[] baseTargets = {
            new Pose2d(13.4724, 3.3073, Rotation2d.fromDegrees(120)), // Tag 6
            new Pose2d(13.8927, 4.0260, Rotation2d.fromDegrees(180)),   // Tag 7
            new Pose2d(13.4724, 4.7460, Rotation2d.fromDegrees(240)),  // Tag 8
            new Pose2d(12.6414, 4.7460, Rotation2d.fromDegrees(300)), // Tag 9
            new Pose2d(12.2287, 4.0260, Rotation2d.fromDegrees(0)), // Tag 10
            new Pose2d(12.6414, 3.3073, Rotation2d.fromDegrees(60)), // Tag 11
            new Pose2d(3.6576, 4.0260, Rotation2d.fromDegrees(0)),  // Tag 17
            new Pose2d(4.9067, 3.3073, Rotation2d.fromDegrees(120)),  // Tag 22
            new Pose2d(5.3209, 4.0260, Rotation2d.fromDegrees(180)),    // Tag 21
            new Pose2d(4.9067, 4.7460, Rotation2d.fromDegrees(240)),   // Tag 20
            new Pose2d(4.0740, 4.7460, Rotation2d.fromDegrees(300)),  // Tag 19
            new Pose2d(4.0740, 3.3073, Rotation2d.fromDegrees(60)),  // Tag 18
        };
    
        Pose2d[] adjustedTargets = new Pose2d[baseTargets.length];
    
        for (int i = 0; i < baseTargets.length; i++) {
            Pose2d tagPose = baseTargets[i];
            double theta = tagPose.getRotation().getRadians(); 
            
            double offsetX = TAG_LEFT_OFFSET_X; 
            double offsetY = TAG_LEFT_OFFSET_Y;
    
            // rotate the offset around theta (rotational matrix)
            double rotatedOffsetX = Math.cos(theta) * offsetX - Math.sin(theta) * offsetY;
            double rotatedOffsetY = Math.sin(theta) * offsetX + Math.cos(theta) * offsetY;
    
            // apply the rotated offset to the tag's position
            double adjustedX = tagPose.getX() - rotatedOffsetX;
            double adjustedY = tagPose.getY() - rotatedOffsetY;
    
            adjustedTargets[i] = new Pose2d(adjustedX, adjustedY, tagPose.getRotation());
        }
    
        return adjustedTargets;
    }
    

    private static Pose2d[] calculateLeftTargetsWithOffset() {
        Pose2d[] baseTargets = {
            new Pose2d(13.4724, 3.3073, Rotation2d.fromDegrees(120)), // Tag 6
            new Pose2d(13.8927, 4.0260, Rotation2d.fromDegrees(180)),   // Tag 7
            new Pose2d(13.4724, 4.7460, Rotation2d.fromDegrees(240)),  // Tag 8
            new Pose2d(12.6414, 4.7460, Rotation2d.fromDegrees(300)), // Tag 9
            new Pose2d(12.2287, 4.0260, Rotation2d.fromDegrees(0)), // Tag 10
            new Pose2d(12.6414, 3.3073, Rotation2d.fromDegrees(60)), // Tag 11
            new Pose2d(3.6576, 4.0260, Rotation2d.fromDegrees(0)),  // Tag 17
            new Pose2d(4.9067, 3.3073, Rotation2d.fromDegrees(120)),  // Tag 22
            new Pose2d(5.3209, 4.0260, Rotation2d.fromDegrees(180)),    // Tag 21
            new Pose2d(4.9067, 4.7460, Rotation2d.fromDegrees(240)),   // Tag 20
            new Pose2d(4.0740, 4.7460, Rotation2d.fromDegrees(300)),  // Tag 19
            new Pose2d(4.0740, 3.3073, Rotation2d.fromDegrees(60)),  // Tag 18
        };
    
        Pose2d[] adjustedTargets = new Pose2d[baseTargets.length];
    
        for (int i = 0; i < baseTargets.length; i++) {
            Pose2d tagPose = baseTargets[i];
            double theta = tagPose.getRotation().getRadians();
            
            double offsetX = -TAG_RIGHT_OFFSET_X;
            double offsetY = TAG_RIGHT_OFFSET_Y;
    
            // rotate the offset around theta (rotational matrix)
            double rotatedOffsetX = Math.cos(theta) * offsetX - Math.sin(theta) * offsetY;
            double rotatedOffsetY = Math.sin(theta) * offsetX + Math.cos(theta) * offsetY;
    
            // apply the rotated offset to the tag's position
            double adjustedX = tagPose.getX() + rotatedOffsetX;
            double adjustedY = tagPose.getY() + rotatedOffsetY;
    
            adjustedTargets[i] = new Pose2d(adjustedX, adjustedY, tagPose.getRotation());
        }
    
        return adjustedTargets;
    }
    
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

    public static Pose2d findClosestTarget(Pose2d currentPose) {
        // Combine your left and right target logic here
        Pose2d leftTarget = findClosestLeftTarget(currentPose);
        Pose2d rightTarget = findClosestRightTarget(currentPose);
        
        if (leftTarget == null) return rightTarget;
        if (rightTarget == null) return leftTarget;
        
        // Return the closer target
        double leftDistance = currentPose.getTranslation().getDistance(leftTarget.getTranslation());
        double rightDistance = currentPose.getTranslation().getDistance(rightTarget.getTranslation());
        
        return leftDistance < rightDistance ? leftTarget : rightTarget;
    }

    // Method to print out the positions
public static void printPositions() {
    System.out.println("Left Positions:");
    for (Pose2d position : LEFT_POSITIONS) {
        System.out.println("X: " + position.getX() + ", Y: " + position.getY() + ", Rotation: " + position.getRotation().getDegrees() + " degrees");
    }

    System.out.println("\nRight Positions:");
    for (Pose2d position : RIGHT_POSITIONS) {
        System.out.println("X: " + position.getX() + ", Y: " + position.getY() + ", Rotation: " + position.getRotation().getDegrees() + " degrees");
    }
}

} 




// OLD TARGETS:


//     // Left Side Positions
//     public static final Pose2d[] LEFT_POSITIONS = {
//         new Pose2d(3.2, 4.18, Rotation2d.fromDegrees(0)),
//         new Pose2d(3.7, 2.99, Rotation2d.fromDegrees(60)),
//         new Pose2d(5.03, 2.8, Rotation2d.fromDegrees(120)),
//         new Pose2d(5.83, 3.87, Rotation2d.fromDegrees(180)),
//         new Pose2d(5.3, 5.07, Rotation2d.fromDegrees(240)),
//         new Pose2d(3.9, 5.2, Rotation2d.fromDegrees(300)),
//         new Pose2d(13.89, 5.06, Rotation2d.fromDegrees(-120)), // RC
//         new Pose2d(12.504, 5.22, Rotation2d.fromDegrees(-60)),// RE
//         new Pose2d(11.673, 4.196, Rotation2d.fromDegrees(0)),//RG
//     new Pose2d(13.58,2.78, Rotation2d.fromDegrees(120)),//RK
//     new Pose2d(12.27,2.95, Rotation2d.fromDegrees(60)),//RI
//         new Pose2d(14.37, 3.89, Rotation2d.fromDegrees(180)),//RA
//         new Pose2d(16.39, 1.03, Rotation2d.fromDegrees(-60)),//RA
//         new Pose2d(16.817, 0.98, Rotation2d.fromDegrees(-57)),//LeftSourceLeft(BLUE)
//     };


//     // Right Side Positions
//     public static final Pose2d[] RIGHT_POSITIONS = {
//         new Pose2d(3.15, 3.8, Rotation2d.fromDegrees(0)),//A
//         new Pose2d(4.0, 2.8, Rotation2d.fromDegrees(60)),//C
//         new Pose2d(5.31, 3.02, Rotation2d.fromDegrees(120)),//E
//         new Pose2d(5.8, 4.21, Rotation2d.fromDegrees(180)),//G
//         new Pose2d(4.96, 5.28, Rotation2d.fromDegrees(240)),//I
//         new Pose2d(3.62, 5.03, Rotation2d.fromDegrees(300)),//K
//         new Pose2d(1.214, 1.006, Rotation2d.fromDegrees(225)),
//         new Pose2d(13.574, 5.307, Rotation2d.fromDegrees(-120)),//RD
//         new Pose2d(12.23, 5.02, Rotation2d.fromDegrees(-60)),//RF
//         new Pose2d(11.682, 3.867, Rotation2d.fromDegrees(0)),//RH
//    new Pose2d(13.89,2.97,Rotation2d.fromDegrees(120)),//RL
//     new Pose2d(12.52,2.79,Rotation2d.fromDegrees(60)),//RJ
//         new Pose2d(14.35, 4.2, Rotation2d.fromDegrees(180)),//RB
//         new Pose2d(16.11, 0.63, Rotation2d.fromDegrees(-54)),//LeftSourceLeft(BLUE)
//     };

    // // Left Side Positions
    // public static final Pose2d[] LEFT_POSITIONS = {
    //     new Pose2d(14.344, 3.867, Rotation2d.fromDegrees(180)),
    //     new Pose2d(13.5, 2.771, Rotation2d.fromDegrees(120)),
    //     new Pose2d(12.220, 3.143, Rotation2d.fromDegrees(60)),
    //     new Pose2d(11.762, 4.187, Rotation2d.fromDegrees(0)),
    //     new Pose2d(12.583, 5.227, Rotation2d.fromDegrees(-60)),
    //     new Pose2d(13.825, 5.065, Rotation2d.fromDegrees(-120)),
    // };

    // // Right Side Positions
    // public static final Pose2d[] RIGHT_POSITIONS = {
    //     new Pose2d(14.358, 4.146, Rotation2d.fromDegrees(180)),
    //     new Pose2d(13.568, 3.264, Rotation2d.fromDegrees(120)),
    //     new Pose2d(12.564, 2.845, Rotation2d.fromDegrees(60)),
    //     new Pose2d(11.746, 3.891, Rotation2d.fromDegrees(0)),
    //     new Pose2d(12.292, 5.076, Rotation2d.fromDegrees(-60)),
    //     new Pose2d(13.537, 5.237, Rotation2d.fromDegrees(-120)),
    // };

 
