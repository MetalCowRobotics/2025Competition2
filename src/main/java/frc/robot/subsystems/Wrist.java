package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkAbsoluteEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.WristConstants;

public class Wrist extends SubsystemBase {
    private final SparkMax wristMotor;
    private final SparkClosedLoopController closedLoopController;
    private final SparkAbsoluteEncoder absoluteEncoder;
    private double targetLocation = 0;
    private double desiredLocation = 0;
    private boolean isInSafePosition = false;

    private double kP = 0.5;
    private double kI = 0.005;
    private double kD = 1.2;

    // private double kP = 0.28;
    // private double kI = 0.265;
    // private double kD = 0.27;

    SparkMaxConfig config;
    AbsoluteEncoderConfig absoluteEncoderConfig;


    public Wrist() {
        wristMotor = new SparkMax(WristConstants.WRIST_MOTOR_ID, MotorType.kBrushless);
        closedLoopController = wristMotor.getClosedLoopController();
        absoluteEncoder = wristMotor.getAbsoluteEncoder();
        absoluteEncoderConfig = new AbsoluteEncoderConfig();
        absoluteEncoderConfig.inverted(true);
  

        this.config = new SparkMaxConfig();
        config.inverted(false)
             .idleMode
             (IdleMode.kBrake)
             .smartCurrentLimit(40)
             .voltageCompensation(12);
        config.softLimit.forwardSoftLimitEnabled(true);
        config.softLimit.reverseSoftLimitEnabled(true);
        config.softLimit.forwardSoftLimit( 0.40);
        config.softLimit.reverseSoftLimit(0.039);
        config.apply(absoluteEncoderConfig);
        config.closedLoop

            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .p(kP)
            .i(kI)
            // .d(kD)
            .iZone(0.07)
            .outputRange(-0.2,0.2)
            .maxMotion
            .maxVelocity(700)
            .maxAcceleration(500)
            .allowedClosedLoopError(.025);
    

        wristMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


        // closedLoopController.setReference(absoluteEncoder.getPosition(), ControlType.kMAXMotionPositionControl);
        this.desiredLocation = 0.04;
    }
    public void setTargetLocation(double targetLocation) {
        this.desiredLocation = targetLocation;
    }

    public void resume() {
        this.targetLocation = this.desiredLocation;
        closedLoopController.setReference(this.targetLocation, ControlType.kMAXMotionPositionControl);
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

    public Command goForward(){
        return this.runOnce(() -> setTargetLocation(WristConstants.Forward_Angle));
    }


    @Override
    public void periodic() {
            resume();
        SmartDashboard.putNumber("Wrist Encoder Reading", wristMotor.getAbsoluteEncoder().getPosition());
        SmartDashboard.putNumber("Wrist Target Location", targetLocation);
    }

    public double getCurrentAngle() {
        return wristMotor.getEncoder().getPosition();
    }
}