package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Climber extends SubsystemBase {
    private final SparkMax climberMotor;
    private static final int CLIMBER_MOTOR_ID = 17; // Adjust this ID as needed
    private static final double CLIMBER_SPEED = 0.5; // 70% speed for intake
    private static final double REVERSE_SPEED = -0.5; // 70% speed for outtake

    public Climber() {
        climberMotor = new SparkMax(CLIMBER_MOTOR_ID, MotorType.kBrushed);

        // Configure the motor
        SparkMaxConfig config = new SparkMaxConfig();

        // Apply Configuration
        climberMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    
    public Command stopIntakeCommand() {
        return this.runOnce(
            () -> climberMotor.set(0)
        );
    }

    public Command startClimberCommand(){
        return this.runOnce(
            () -> climberMotor.set(CLIMBER_SPEED)
        );
    }

    public Command reverseClimberCommand() {
        return this.runOnce(
            () -> climberMotor.set(REVERSE_SPEED)
        );
    }
}
