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
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElevatorConstants;


public class Elevator extends SubsystemBase {
    private final SparkMax elevatorMotor;
    private final SparkMax elevatorFollowerMotor;
    private final SparkClosedLoopController closedLoopController;
    private final SparkLimitSwitch bottomSwitch;
    private final SparkLimitSwitch topSwitch;
    private double targetLocation = 0;
    private final Wrist wrist;
    private double desiredTarget = 0;

    public Elevator(Wrist wrist) {
        this.wrist = wrist;
        elevatorMotor = new SparkMax(ElevatorConstants.ELEVATOR_MOTOR_ID, MotorType.kBrushless);
        elevatorFollowerMotor = new SparkMax(ElevatorConstants.ELEVATOR_FOLLOWER_MOTOR_ID, MotorType.kBrushless);
        
        closedLoopController = elevatorMotor.getClosedLoopController();
        bottomSwitch = elevatorMotor.getReverseLimitSwitch();
        topSwitch = elevatorMotor.getForwardLimitSwitch();

        // Configure the main elevator motor
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(true);  // Main motor is inverted
        config.idleMode(IdleMode.kBrake)
             .smartCurrentLimit(40)
             .voltageCompensation(12);

        config.limitSwitch
            .reverseLimitSwitchEnabled(true)
            .reverseLimitSwitchType(Type.kNormallyOpen)
            .forwardLimitSwitchEnabled(true)
            .forwardLimitSwitchType(Type.kNormallyOpen);

        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(1)
            // .d(0.001)
            .i(0.0001)
            // .iZone(4)
            .outputRange(-1, 1)
            .maxMotion
            .maxVelocity(6000)
            .maxAcceleration(5000)
            .allowedClosedLoopError(0.5);

        elevatorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Configure the follower motor
        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig.inverted(false); 
        followerConfig.follow(15, true); // Changed to false since main motor is inverted
        followerConfig.idleMode(IdleMode.kCoast)
                     .smartCurrentLimit(40)
                     .voltageCompensation(12);
        elevatorFollowerMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        followerConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(1)
            // .d(0.001)
            .i(0.0001)
            // .iZone(4)
            .outputRange(-1, 1)
            .maxMotion
            .maxVelocity(6000)
            .maxAcceleration(5000)
            .allowedClosedLoopError(0.5);
        
        // Set follower motor to follow the main motor in opposite direction
        elevatorFollowerMotor.isFollower(); // true inverts the following direction
       
        zeroEncoder();
    }

    public void setTargetLocation(double targetLocation) {
        this.desiredTarget = targetLocation;
    }

    private void zeroEncoder() {
        elevatorMotor.getEncoder().setPosition(0);
    }

    // Command wrappers for the preset positions
    public Command goToL4Command() {
        return this.runOnce(() -> setTargetLocation(ElevatorConstants.L4_Distance));
    }

    public Command goToL3Command() {
        return this.runOnce(() -> setTargetLocation(ElevatorConstants.L3_Distance));
    }

    public Command goToL2Command() {
        return this.runOnce(() -> setTargetLocation(ElevatorConstants.L2_Distance));
    }

    public Command goToSourceCommand() {
        return this.runOnce(() -> setTargetLocation(ElevatorConstants.Source_Distance));
    }

    public Command goToRestCommand() {
        return this.runOnce(() -> setTargetLocation(ElevatorConstants.resetPos));
    }

    @Override
    public void periodic() {
        // If elevator at or below L3 but wanting to go to L4, make sure wrist is tucked in (>=11.5)
        // If elevator is at L4 but wanting to go below L4, wait until wrist is tucked in (>= 11.5)
        boolean atOrBelowL3 = (getPosition() - ElevatorConstants.L3_Distance) < 3.0;

        if (desiredTarget > ElevatorConstants.L3_Distance && atOrBelowL3) {
            if (wrist.isAtSafeAngle()) {
                // wrist tucked so we can to L4
                this.targetLocation = desiredTarget;
                closedLoopController.setReference(this.targetLocation, ControlType.kMAXMotionPositionControl);
                wrist.resume();
            } else {
                // wanting to go to L4 but wrist is out so wait for next periodic
                wrist.tuck();
            }
        } else if (desiredTarget <= ElevatorConstants.L4_Distance && !atOrBelowL3) {
            if (wrist.isAtSafeAngle()) {
                this.targetLocation = desiredTarget;
                closedLoopController.setReference(desiredTarget, ControlType.kMAXMotionPositionControl);
                wrist.resume();
            } else {
                // wait until wrist tucked before lowering from L4 to L3 or below
                wrist.tuck();
            }
        } else {
            this.targetLocation = desiredTarget;
            closedLoopController.setReference(this.targetLocation, ControlType.kMAXMotionPositionControl);
            wrist.resume();
        }

        printDashboard();

        SmartDashboard.putNumber("Elevator/Follower Current", elevatorFollowerMotor.getOutputCurrent());
        SmartDashboard.putNumber("Elevator/Follower Temperature", elevatorFollowerMotor.getMotorTemperature());
    }

    public void printDashboard() {
        SmartDashboard.putNumber("Elevator Position", elevatorMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Elevator Target", targetLocation);
        SmartDashboard.putNumber("Elevator Desired Target", desiredTarget);
        SmartDashboard.putBoolean("Elevator Bottom Switch", bottomSwitch.isPressed());
        SmartDashboard.putBoolean("Elevator Top Switch", topSwitch.isPressed());
    }

    private double getPosition() {  
        return elevatorMotor.getEncoder().getPosition();
    }
} 