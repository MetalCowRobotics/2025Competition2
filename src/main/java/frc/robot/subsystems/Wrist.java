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
    private final double SAFE_ANGLE = 0.4;
    private boolean isInSafePosition = false;

    public Wrist() {
        wristMotor = new SparkMax(WristConstants.WRIST_MOTOR_ID, MotorType.kBrushless);
        closedLoopController = wristMotor.getClosedLoopController();
        
        // Configure the absolute encoder
        absoluteEncoder = wristMotor.getAbsoluteEncoder();
        
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(true);
        config.idleMode(IdleMode.kCoast)
             .smartCurrentLimit(50)
             .voltageCompensation(12);

        config.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder) // Change to use absolute encoder
            .p(0.1) // You may need to retune these PID values
            .d(0.08)
            .i(0.00006)
            .iZone(0.01)
            .outputRange(-0.3, 0.3)
            .maxMotion
            .maxVelocity(4200)
            .maxAcceleration(4000)
            .allowedClosedLoopError(.01);

        wristMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setTargetLocation(double targetLocation) {
        this.desiredLocation = targetLocation;
    }

    public void tuck() {
        closedLoopController.setReference(WristConstants.L4_Angle, ControlType.kMAXMotionPositionControl);
    }

    public void resume() {
        this.targetLocation = this.desiredLocation;
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