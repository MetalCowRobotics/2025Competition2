package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.WristConstants;

public class Climb extends SubsystemBase {
    private final SparkMax climbMotor;
    private static final double CLIMB_SPEED = 0.5; // 70% speed for intake


    public Climb() {
        climbMotor = new SparkMax(21, MotorType.kBrushed);

        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(false);
        config.idleMode(IdleMode.kCoast)
             .smartCurrentLimit(50)
             .voltageCompensation(12);

        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(0.1)
            .d(0.08)
            .i(0.00006)
            .iZone(0.5)
            .outputRange(-0.5, 0.5)
            .maxMotion
            .maxVelocity(4200)
            .maxAcceleration(4000)
            .allowedClosedLoopError(.25);

        climbMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command startIntakeCommand() {
        return this.startEnd(
            // When the command starts, run the intake
            () -> climbMotor.set(CLIMB_SPEED),
            // When the command ends, stop the intake
            () -> climbMotor.set(0)
        );
    }

    public Command reverseIntakeCommand() {
        return this.startEnd(
            // When the command starts, run the intake
            () -> climbMotor.set(-CLIMB_SPEED),
            // When the command ends, stop the intake
            () -> climbMotor.set(0)
        );
    }

    @Override
    public void periodic() {
    }
}
