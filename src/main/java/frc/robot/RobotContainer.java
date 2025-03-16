// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.AlignToTarget;
import frc.robot.commands.ArmCommands;
import frc.robot.commands.LEDDefaultCommand;
import frc.robot.constants.AlignmentConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Wrist;

public class RobotContainer {
    /* Constants */
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)/2; // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.5).in(RadiansPerSecond); // 1/2 of a rotation per second


    private double CrawlMaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)/4; // kSpeedAt12Volts desired top speed
    private double CrawlMaxAngularRate = RotationsPerSecond.of(0.5).in(RadiansPerSecond)/2; // 1/2 of a rotation per second

    /* Drive Request Objcets */
    private final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1)// Accounts for 10% deadband from the movement joystick
            .withRotationalDeadband(MaxAngularRate * 0.1) // Accounts for 30% deadband from the rotational joystick
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors making it respond from raw inputs
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    /* Controllers */
    private final CommandXboxController driverController = new CommandXboxController(0); // Driver controller
    private final CommandXboxController operatorController = new CommandXboxController(1); // Operator controller

    /* Vision */
    private final Vision vision;
    private final LEDSubsystem ledSubsystem = new LEDSubsystem(0);

    /* Path follower */
    private final SendableChooser<Command> autoChooser;
    private final SendableChooser<String> autoLocationChooser;

    /* Subsystems */
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final Intake intake = new Intake();
    private final Wrist wrist = new Wrist();
    private final Elevator elevator = new Elevator(wrist);
    private final Climb climb = new Climb();
    double targetPosition = 0;

    /* Commands */
    private final ArmCommands armCommands;

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
        NamedCommands.registerCommand("Stall Intake", intake.stallIntakeCommand());
        NamedCommands.registerCommand("Wrist Out", wrist.goForwardCommand());
        NamedCommands.registerCommand("L3 Alage Remove", armCommands.goToAlgaeL3());
        NamedCommands.registerCommand("L2 Alage Remove", armCommands.goToAlgaeL2());
        NamedCommands.registerCommand("Intake Hold", intake.startIntakeCommand());

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Mode", autoChooser);
        autoLocationChooser = new SendableChooser<>();
        autoLocationChooser.addOption("Left", new String("Left"));
        autoLocationChooser.addOption("Center", new String("Center"));
        autoLocationChooser.addOption("Right", new String("Right"));

        SmartDashboard.putData("Auto Location", autoLocationChooser);

        // Set default command for LEDs
        ledSubsystem.setDefaultCommand(new LEDDefaultCommand(
            ledSubsystem, 
            drivetrain, 
            driverController, 
            operatorController
        ));

        configureBindings();
    }

    private void configureBindings() {



        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {

                double distanceToReef = new Translation2d(
                    drivetrain.getState().Pose.getX(), 
                    drivetrain.getState().Pose.getY()
                ).getDistance(
                    new Translation2d(
                        AlignmentConstants.findClosestTarget(drivetrain.getState().Pose).getX(), 
                        AlignmentConstants.findClosestTarget(drivetrain.getState().Pose).getY()
                    )
                );

                if (driverController.rightTrigger().getAsBoolean() || distanceToReef <= 0.4) {
                    return fieldCentricDrive.withVelocityX(-driverController.getLeftY() * CrawlMaxSpeed) 
                                            .withVelocityY(-driverController.getLeftX() * CrawlMaxSpeed) 
                                            .withRotationalRate(-driverController.getRightX() * CrawlMaxAngularRate);
                } else {
                    return fieldCentricDrive.withVelocityX(-driverController.getLeftY() * MaxSpeed) 
                                            .withVelocityY(-driverController.getLeftX() * MaxSpeed) 
                                            .withRotationalRate(-driverController.getRightX() * MaxAngularRate);
                }
            })
        );
 
        // Left Align Button 
        driverController.x().whileTrue(
            new AlignToTarget(drivetrain, () -> {
                var currentPose = drivetrain.getState().Pose;
                return AlignmentConstants.findClosestLeftTarget(currentPose);
            })
        );

        // Right Align Button
        driverController.b().whileTrue(
            new AlignToTarget(drivetrain, () -> {
                var currentPose = drivetrain.getState().Pose;
                return AlignmentConstants.findClosestRightTarget(currentPose);
            })
        );

        // Robot Centric Drive If Requested
        driverController.pov(0).whileTrue(drivetrain.applyRequest(() ->
            robotCentricDrive.withVelocityX(0.5).withVelocityY(0))
        );
        driverController.pov(180).whileTrue(drivetrain.applyRequest(() ->
            robotCentricDrive.withVelocityX(-0.5).withVelocityY(0))
        );

        /* Operator Commands */

        // Makes L4 Only Available When 1 Meter Close to Reef

        // not using currently since robot doesn't have limelights on it
        // operatorController.y().onTrue(
        //     new InstantCommand(() -> {
        //         if (distance < 1.25) {
        //             armCommands.goToL4();
        //         }
        //     })
        // );

        operatorController.y().onTrue(armCommands.goToL4());
        
        // L4 position on Y
        operatorController.x().onTrue(armCommands.goToSource()); // Source position on X
        operatorController.b().onTrue(armCommands.goToL3());     // L4 position on Y
        operatorController.a().onTrue(armCommands.goToL2());

        operatorController.rightTrigger().onTrue(armCommands.goToAlgaeL2()); // Remove algae from L2 on Right Trigger
        operatorController.leftTrigger().onTrue(armCommands.goToAlgaeL3()); // Remove algae from L3 on Right Trigger

        // Manual Control On Intake
        operatorController.leftBumper().onTrue(intake.reverseIntakeCommand()); // Release Coral on Left Bumper
        operatorController.rightBumper().onTrue(intake.stallIntakeCommand());  // Keep Coral In on Right Bumper
        operatorController.start().onTrue(intake.stopIntakeCommand());
        
        // Rest position on driver left bumper
        driverController.leftBumper().onTrue(armCommands.goToRest()); // Home Position

        // Climb Control
        driverController.a().whileTrue(climb.runClimb());
        driverController.y().whileTrue(climb.reverseClimb());

        // Telementry Update
        drivetrain.registerTelemetry(logger::telemeterize);

        // Manual Controls for Elevator
        while (operatorController.getLeftY() > 0.3){
            double currentElevatorPosition = elevator.getPosition();
            targetPosition = currentElevatorPosition + 0.2;
            
            elevator.runOnce(() -> elevator.setTargetLocation(targetPosition));
            elevator.elevatorMoveToDesired();
        }

        while (operatorController.a().getAsBoolean()){
            double currentElevatorPosition = elevator.getPosition();
            targetPosition = currentElevatorPosition - 1;


            elevator.setTargetLocation(targetPosition);
            elevator.elevatorMoveToDesired();
        }

        SmartDashboard.putNumber("Manual New Target", targetPosition);



        // Manual Control on Elevator
        // if(operatorController.getLeftY()>0.1){
        //     elevator.setUp();
        // }

        // if(operatorController.getLeftY()<-0.1){
        //     elevator.setDown();
        // }
        
        // if(operatorController.start().getAsBoolean()){
        //     elevator.zeroEncoder();
        // }
    }

    public Command getAutonomousCommand() {
        try{
            if(autoLocationChooser.getSelected().equals("Right")){
                return new PathPlannerAuto(autoChooser.getSelected().getName(), true);
            }else{
                return new PathPlannerAuto(autoChooser.getSelected().getName(), false);
            }
        }catch(Exception e){
                DriverStation.reportError("PathPlanner ERROR: " + e.getMessage(), e.getStackTrace());
                return null;
        }
    }
}
