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
import com.revrobotics.AbsoluteEncoder;
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
    private final double SAFE_ANGLE = 0.35;
    private boolean isInSafePosition = false;
    // private double kP = 0.09;
    // private double kI = 0.12;
    // private double kD = 0.1;

    private double kP = 0.13;
    private double kI = 0.08;
    private double kD = 0.09;


    public Wrist() {
        wristMotor = new SparkMax(WristConstants.WRIST_MOTOR_ID, MotorType.kBrushless);
        closedLoopController = wristMotor.getClosedLoopController();
        
        // Configure the absolute encoder
        absoluteEncoder = wristMotor.getAbsoluteEncoder();
        
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(true);
        config.idleMode(IdleMode.kBrake)
             .smartCurrentLimit(50)
             .voltageCompensation(12);

        // Enable soft limits
        config.softLimit
           .forwardSoftLimit(0.75);

        config.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .p(kP)
            .d(kD)
            .i(kI)
            .iZone(0.05)
            .outputRange(-0.4, 0.4)
            .maxMotion
            .maxVelocity(4200)
            .maxAcceleration(4000)
            // .allowedClosedLoopError(.015);
            .allowedClosedLoopError(.05);

        wristMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setTargetLocation(double targetLocation) {
        // Constrain target location between 0 and 0.75
        this.desiredLocation = Math.min(Math.max(targetLocation, 0), 0.75);
    }

    public void tuck() {
        closedLoopController.setReference(WristConstants.L4_Angle, ControlType.kMAXMotionPositionControl);
    }

    public void resume() {
        this.targetLocation = this.desiredLocation;
        closedLoopController.setReference(this.targetLocation, ControlType.kMAXMotionPositionControl);
    }

    public void holdLastTarget(){
        closedLoopController.setReference(this.targetLocation, ControlType.kMAXMotionPositionControl);
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

    public Command goForwardCommand() {
        return this.runOnce(() -> setTargetLocation(WristConstants.Forward_Angle));
    }

    public Command goToAlage(){
        return this.runOnce(() -> setTargetLocation(WristConstants.Alage_Angle));
    }

    public boolean isAtTarget(){
        return Math.abs(absoluteEncoder.getPosition() - targetLocation) < 0.02;
    }

    @Override
    public void periodic() {
        // Update safety status
        isInSafePosition = getCurrentAngle() >= SAFE_ANGLE;

        // Encoder information
        SmartDashboard.putNumber("Wrist/Absolute Position", absoluteEncoder.getPosition());
        SmartDashboard.putNumber("Wrist/Absolute Velocity", absoluteEncoder.getVelocity());
        SmartDashboard.putNumber("Wrist/Target Location", targetLocation);
        SmartDashboard.putNumber("Wrist/Desired Location", desiredLocation);
        SmartDashboard.putBoolean("Wrist/Is Safe Position", isInSafePosition);
        SmartDashboard.putNumber("Wrist/Safe Angle Threshold", SAFE_ANGLE);
        SmartDashboard.putNumber("Wrist/Forward Limit", 0.75);
        
        // Motor information
        SmartDashboard.putNumber("Wrist/Motor Output", wristMotor.getAppliedOutput());
        SmartDashboard.putNumber("Wrist/Motor Current", wristMotor.getOutputCurrent());
    }

    public double getCurrentAngle() {
        return absoluteEncoder.getPosition();
    }

    public boolean isAtSafeAngle() {
        return isInSafePosition;
    }
} 