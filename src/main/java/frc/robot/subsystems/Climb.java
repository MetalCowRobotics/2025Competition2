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
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimbConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.WristConstants;

public class Climb extends SubsystemBase {
    private final SparkMax climbMotor;
    private final SparkClosedLoopController closedLoopController;
    private double targetLocation = 0;
    private boolean isAtLockInState;
    private final SparkLimitSwitch climbSwitch;
    private double climbMoterSpeed;

    public Climb() {
        climbMotor = new SparkMax(ClimbConstants.CLIMB_MOTOR_ID, MotorType.kBrushless);
        climbSwitch = climbMotor.getForwardLimitSwitch();
        climbMoterSpeed = 0;
        isAtLockInState = false;
        closedLoopController = climbMotor.getClosedLoopController();

        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(true);
        config.limitSwitch.forwardLimitSwitchEnabled(false);
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
        zeroEncoder();
    }

    public void setTargetLocation(double targetLocation) {
        this.targetLocation = targetLocation;
        closedLoopController.setReference(targetLocation, ControlType.kMAXMotionPositionControl);
    }

    private void zeroEncoder() {
        climbMotor.getEncoder().setPosition(0);
    }


    public boolean isLimitSwitchPressed() {
        return climbSwitch.isPressed();
    }

    public void stopClimbMotor() {
        setTargetLocation(climbMotor.getEncoder().getPosition());
       }

    public Command lockIn() {
 
        return this.runOnce(() -> setTargetLocation(ClimbConstants.LOCK_IN_ROTATION));
    }

    public Command retrieve() {
        if(isAtLockInState){
            isAtLockInState = false;
            return this.runOnce(() -> setTargetLocation(ClimbConstants.RETRIEVE_ROTATION));
        }else{
            return null;
        }
    }

    @Override
    public void periodic() {

        if(isLimitSwitchPressed()){
            isAtLockInState = true;
            stopClimbMotor();
        }
        SmartDashboard.putNumber("Climb Encoder Reading", climbMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Climb Target Location", targetLocation);
    }
} 