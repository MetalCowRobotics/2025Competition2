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

public class Wrist extends SubsystemBase {
    private final SparkMax wristMotor;
    private final SparkClosedLoopController closedLoopController;
    private double targetLocation = 0;

    public Wrist() {
        wristMotor = new SparkMax(WristConstants.WRIST_MOTOR_ID, MotorType.kBrushless);
        closedLoopController = wristMotor.getClosedLoopController();

        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(true);
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

        wristMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        zeroEncoder();
    }

    public void setTargetLocation(double targetLocation) {
        this.targetLocation = targetLocation;
        closedLoopController.setReference(targetLocation, ControlType.kMAXMotionPositionControl);
    }

    private void zeroEncoder() {
        wristMotor.getEncoder().setPosition(0);
    }

    // Command wrappers for the preset positions
    public Command goToL4Command() {
        return this.runOnce(() -> setTargetLocation(WristConstants.L4_Angle));
    }

    public Command goToL3Command() {
        return this.runOnce(() -> setTargetLocation(WristConstants.L3_Angle));
    }

    public Command goToSourceCommand() {
        return this.runOnce(() -> setTargetLocation(WristConstants.Source_Angle));
    }

    public Command goToRestCommand() {
        return this.runOnce(() -> setTargetLocation(WristConstants.Rest_Angle));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Wrist Encoder Reading", wristMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Wrist Target Location", targetLocation);
    }
} 