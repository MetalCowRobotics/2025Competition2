package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AlignmentConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class AlignToTarget extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final Pose2d targetPose;
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController rotationController;
    private final SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric();

    public AlignToTarget(CommandSwerveDrivetrain drivetrain, Pose2d targetPose) {
        this.drivetrain = drivetrain;
        this.targetPose = targetPose;
        
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
        xController.setSetpoint(targetPose.getX());
        yController.setSetpoint(targetPose.getY());
        rotationController.setSetpoint(targetPose.getRotation().getRadians());
    }

    @Override
    public void execute() {
        var currentPose = drivetrain.getState().Pose;
        
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
    }

    @Override
    public boolean isFinished() {
        return xController.atSetpoint() &&
               yController.atSetpoint() &&
               rotationController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(fieldCentric.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
    }
} 