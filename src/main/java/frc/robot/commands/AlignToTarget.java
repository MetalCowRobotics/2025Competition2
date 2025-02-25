package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AlignmentConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.function.Supplier;

public class AlignToTarget extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final Supplier<Pose2d> targetPoseSupplier;
    private Pose2d targetPose;
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController rotationController;
    private final SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric();
    private static final double ACTIVATION_DISTANCE_METERS = 1.0;

    public AlignToTarget(CommandSwerveDrivetrain drivetrain, Supplier<Pose2d> targetPoseSupplier) {
        this.drivetrain = drivetrain;
        this.targetPoseSupplier = targetPoseSupplier;
        
        xController = new PIDController(
            AlignmentConstants.XY_P,
            AlignmentConstants.XY_I,
            AlignmentConstants.XY_D
        );
        yController = new PIDController(
            AlignmentConstants.XY_P,
            AlignmentConstants.XY_I,
            AlignmentConstants.XY_D
        );
        rotationController = new PIDController(
            AlignmentConstants.ROTATION_P,
            AlignmentConstants.ROTATION_I,
            AlignmentConstants.ROTATION_D
        );

        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        
        xController.setTolerance(AlignmentConstants.POSITION_TOLERANCE_METERS);
        yController.setTolerance(AlignmentConstants.POSITION_TOLERANCE_METERS);
        rotationController.setTolerance(Math.toRadians(AlignmentConstants.ROTATION_TOLERANCE_DEGREES));

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        targetPose = targetPoseSupplier.get();
        xController.setSetpoint(targetPose.getX());
        yController.setSetpoint(targetPose.getY());
        rotationController.setSetpoint(targetPose.getRotation().getRadians());
    }

    @Override
    public boolean isFinished() {
        var currentPose = drivetrain.getState().Pose;
        
        // Calculate distance to target
        double distance = new Translation2d(
            currentPose.getX(), 
            currentPose.getY()
        ).getDistance(
            new Translation2d(
                targetPose.getX(), 
                targetPose.getY()
            )
        );

        // If we're too far away, end the command
        if (distance > ACTIVATION_DISTANCE_METERS) {
            return true;
        }

        // Otherwise, check if we've reached the target
        return xController.atSetpoint() &&
               yController.atSetpoint() &&
               rotationController.atSetpoint();
    }

    @Override
    public void execute() {
        var currentPose = drivetrain.getState().Pose;
        
        // Calculate distance to target
        double distance = new Translation2d(
            currentPose.getX(), 
            currentPose.getY()
        ).getDistance(
            new Translation2d(
                targetPose.getX(), 
                targetPose.getY()
            )
        );

        // Only run alignment if we're within activation distance
        if (distance <= ACTIVATION_DISTANCE_METERS) {
            double xSpeed = xController.calculate(currentPose.getX());
            double ySpeed = yController.calculate(currentPose.getY());
            double rotationSpeed = rotationController.calculate(
                currentPose.getRotation().getRadians()
            );

            // Clamp speeds to reasonable values
            xSpeed = MathUtil.clamp(xSpeed, -2.0, 2.0);
            ySpeed = MathUtil.clamp(ySpeed, -2.0, 2.0);
            rotationSpeed = MathUtil.clamp(rotationSpeed, -2.0, 2.0);

            drivetrain.setControl(
                fieldCentric
                    .withVelocityX(xSpeed)
                    .withVelocityY(ySpeed)
                    .withRotationalRate(rotationSpeed)
            );
        } else {
            // Stop if we're too far
            drivetrain.setControl(
                fieldCentric
                    .withVelocityX(0)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            );
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(fieldCentric.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
    }
} 