package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
    private final SparkMax climbMotor;
    private final SparkClosedLoopController closedLoopController;
    private final DigitalInput limitSwitch;
    private double targetPosition = 0;
    private static final int MOTOR_ID = 18;
    private static final double CLIMB_SPEED = 1; // 30% speed
    private static final double ADDITIONAL_ROTATIONS = 330;

    public Climb() {
        climbMotor = new SparkMax(MOTOR_ID, MotorType.kBrushless);
        closedLoopController = climbMotor.getClosedLoopController();
        limitSwitch = new DigitalInput(0); // DIO port 0

        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(false);
        config.idleMode(IdleMode.kBrake)
             .smartCurrentLimit(40)
             .voltageCompensation(12);

        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(5)
            .d(0)
            .i(0)
            .outputRange(-0, 1)
            .maxMotion
            .maxVelocity(5000)
            .maxAcceleration(2000)
            .allowedClosedLoopError(0.1);

        climbMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command runClimb() {
        return this.startEnd(
            // When command starts
            () -> climbMotor.set(CLIMB_SPEED),
            // When command ends
            () -> {
                climbMotor.set(0);

            }
        );
    }

    // public Command runClimb(){
    //     return this.runOnce(() -> {climbMOtor.set(CLIMB_SPEED)})
    // }

    // public Command climbAdditionalCommand() {

    //     return this.runOnce(() -> {
    //         targetPosition = ADDITIONAL_ROTATIONS;
    //         closedLoopController.setReference(targetPosition, ControlType.kMAXMotionPositionControl);
    //     });
    // }

    private void zeroEncoder() {
        climbMotor.getEncoder().setPosition(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climb Position", climbMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Climb Target", targetPosition);
        SmartDashboard.putBoolean("Climb Limit Switch", !limitSwitch.get());
    }
} 