package frc.robot.subsystems;

import frc.robot.Robot;

import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class Intake extends SubsystemBase {
    private final SparkMax intakeMotor;
    private static final int INTAKE_MOTOR_ID = 17; // Adjust this ID as needed
    private static final double INTAKE_SPEED = 0.85; // 70% speed for intake
    private static final double REVERSE_SPEED = -1; // 70% speed for outtake
    private static final double STALL_SPEED = 0.3;
    private static final double SLOW_REVERSE_SPEED = -0.65;
    public static SparkLimitSwitch limitSwitch;
    public         SparkMaxConfig config;

    public Intake() {
        intakeMotor = new SparkMax(INTAKE_MOTOR_ID, MotorType.kBrushed);
        
        // Configure the motor
        limitSwitch = intakeMotor.getForwardLimitSwitch(); 
        config = new SparkMaxConfig();
        config.limitSwitch.forwardLimitSwitchEnabled(false);

        intakeMotor.configure(config, null, null);

    }

    public Command startIntakeCommand() {
        return this.runOnce(
            () -> intakeMotor.set(INTAKE_SPEED)
        );
    }
    public Command stallIntakeCommand() {
        return this.runOnce(
            () -> intakeMotor.set(STALL_SPEED));
        
    }

    public Command reverseIntakeCommand() {
        return this.runOnce(
            () -> intakeMotor.set(REVERSE_SPEED)
        );
    }

    public Command slowReverseIntakeCommand() {
        return this.runOnce(
            () -> intakeMotor.set(SLOW_REVERSE_SPEED)
        );
    }

    public Command stopIntakeCommand() {
        return this.runOnce(
            () -> intakeMotor.set(0)
        );
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void stop() {
        intakeMotor.set(0);
    }
} 