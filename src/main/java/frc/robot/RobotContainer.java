// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.constants.AlignmentConstants;
import frc.robot.commands.AlignToTarget;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Elevator;
import frc.robot.commands.ArmCommands;
import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.commands.LEDDefaultCommand;
import frc.robot.subsystems.Climb;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)/2; // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.5).in(RadiansPerSecond); // 1/2 of a rotation per second

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0); // Driver controller
    private final CommandXboxController operatorController = new CommandXboxController(1); // Operator controller

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final Vision vision;

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    private final Intake intake = new Intake();
    private final Wrist wrist = new Wrist();
    private final Elevator elevator = new Elevator(wrist);

    private final ArmCommands armCommands;

    private final LEDSubsystem ledSubsystem = new LEDSubsystem(0);

    private final Climb climb = new Climb();

    public RobotContainer() {
        // Create vision subsystem after drivetrain
        vision = new Vision(drivetrain);
        
        // Initialize arm commands
        armCommands = new ArmCommands(elevator, wrist, intake);
        
        // Register Named Commands for PathPlanner
        NamedCommands.registerCommand("Go To L2", armCommands.goToL2());
        NamedCommands.registerCommand("Go To L3", armCommands.goToL3());
        NamedCommands.registerCommand("Go To L4", armCommands.goToL4());
        NamedCommands.registerCommand("Go To Source", armCommands.goToSource());
        NamedCommands.registerCommand("Go To Rest", armCommands.goToRest());
        NamedCommands.registerCommand("Stop All", armCommands.stopAll());
        NamedCommands.registerCommand("Start Intake", intake.startIntakeCommand());
        NamedCommands.registerCommand("Stop Intake", intake.stopIntakeCommand());
        NamedCommands.registerCommand("Reverse Intake", intake.reverseIntakeCommand());
        
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Mode", autoChooser);

        // Set default command for LEDs
        ledSubsystem.setDefaultCommand(new LEDDefaultCommand(
            ledSubsystem, 
            drivetrain, 
            joystick, 
            operatorController
        ));

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        
        // X button for left side targets with LED feedback
        joystick.x().whileTrue(
            new AlignToTarget(drivetrain, () -> {
                var currentPose = drivetrain.getState().Pose;
                return AlignmentConstants.findClosestLeftTarget(currentPose);
            })
        );

        // B button for right side targets with LED feedback
        joystick.b().whileTrue(
            new AlignToTarget(drivetrain, () -> {
                var currentPose = drivetrain.getState().Pose;
                return AlignmentConstants.findClosestRightTarget(currentPose);
            })
        );

        joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );

        // Combined elevator and wrist controls for all positions
        operatorController.a().onTrue(armCommands.goToL2());     // L2 position on A
        operatorController.b().onTrue(armCommands.goToL3());     // L3 position on B
        operatorController.y().onTrue(armCommands.goToL4());     // L4 position on Y
        operatorController.x().onTrue(armCommands.goToSource()); // Source position + intake on X
        joystick.leftBumper().onTrue(armCommands.goToRest());   // Rest position on driver left bumper

        // Stop intake
        operatorController.leftBumper().onTrue(intake.reverseIntakeCommand());
        
        // Reverse intake - toggle style
        operatorController.rightBumper().onTrue(
                intake.stopIntakeCommand()
        );

        drivetrain.registerTelemetry(logger::telemeterize);

        // Climb controls
        joystick.a().whileTrue(climb.runUntilLimitCommand());
        joystick.y().onTrue(climb.climbAdditionalCommand());
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
