// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ElectronicsIDs;
import frc.robot.Constants.LogitechExtreme3DConstants;
//import frc.robot.Constants.LogitechDAConstants;
//import frc.robot.Constants.RadioMasterConstants;
import frc.robot.Constants.XboxControllerConstants;
import frc.robot.commands.DriveRobot;
import frc.robot.commands.DriveRobotWithAlign;
import frc.robot.commands.SetShooterMountPosition;
import frc.robot.commands.StartShooterIntake;
import frc.robot.commands.StopShooterIntake;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.FloorIntake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterMount;
import frc.robot.subsystems.ShooterMount.ShooterMountState;
import frc.robot.subsystems.Underglow;
import frc.robot.subsystems.Vision;

public class RobotContainer {

    /* SUBSYSTEMS */

    public final Vision visionSub = new Vision();
    public final Drive driveSub = new Drive(visionSub);
    public final Shooter shooterSub = new Shooter();
    public final ShooterMount shooterMountSub = new ShooterMount();
    public final FloorIntake floorIntakeSub = new FloorIntake();
    public final Underglow underglowSub = new Underglow();

    /* COMMANDS */

    private final SetShooterMountPosition setShooterPosSpeakerCmd = new SetShooterMountPosition(shooterMountSub,
            ShooterMountState.Speaker, visionSub);
    private final SetShooterMountPosition setShooterPosAmpCmd = new SetShooterMountPosition(shooterMountSub,
            ShooterMountState.Amp, visionSub);
    private final SetShooterMountPosition setShooterPosSourceIntakeCmd = new SetShooterMountPosition(shooterMountSub,
            ShooterMountState.SourceIntake, visionSub);
    public final SetShooterMountPosition setShooterPosFloorIntakeCmd = new SetShooterMountPosition(shooterMountSub,
            ShooterMountState.FloorIntake, visionSub);
    private final SetShooterMountPosition setShooterPosTrapCmd = new SetShooterMountPosition(shooterMountSub,
            ShooterMountState.Trap, visionSub);

    private final StartShooterIntake startShooterIntakeCmd = new StartShooterIntake(shooterSub, floorIntakeSub,
            shooterMountSub);
    private final StopShooterIntake stopShooterIntakeCmd = new StopShooterIntake(shooterSub, floorIntakeSub);

    // private final Climb climbCmd = new Climb(shooterPositionSub);
    // LT Climb
    private final SetShooterMountPosition prepareToClimbCmd = new SetShooterMountPosition(shooterMountSub,
            ShooterMountState.PreClimb, visionSub);
    private final SetShooterMountPosition climbCmd = new SetShooterMountPosition(shooterMountSub,
            ShooterMountState.Climb, visionSub);


    /* CONTROLLERS */

    Joystick driverController;
    Joystick operatorController;

    /* BUTTONS */
    private Trigger moveToSpeakerButton;
    private Trigger moveToAmpButton;
    private Trigger moveToSourceButton;
    private Trigger moveToFloorButton;
    private Trigger moveToTrapButton;
    private Trigger prepareToClimbButton; // LT added. CHANGE if not its own buttun
    public Trigger shootButton;
    private Trigger climbButton;
    private Trigger LEDHumanSourceButton;
    private Trigger LEDHumanFloorButton;
    private Trigger shootIntakeButton;
    private Trigger autoAlignButton;

    /* AUTO */

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        AutoBuilder.configureHolonomic(
                // driveSub::getPoseWithoutVision, // Robot pose supplier
                driveSub::getPoseWithVision, // Robot pose supplier
                driveSub::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                driveSub::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                driveSub::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig(
                        new PIDConstants(5, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5, 0.0, 0.0), // Rotation PID constants
                        4.59, // Max module speed, in m/s
                        0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                driveSub);
        configurePathPlannerLogging();
        // NamedCommands.registerCommand(
        // "Shoot Speaker",
        // setShooterSpeakerAngleCmd
        // );

        // NamedCommands.registerCommand(
        // "Shoot Amp",
        // setShooterAmpAngleCmd
        // );

        NamedCommands.registerCommand("Shoot Speaker",
                Commands.print("***********************************Shoot into Speaker"));
        // NamedCommands.registerCommand("Shoot Speaker",
        // Commands.runOnce(moveShooterToAmpCommand));
        NamedCommands.registerCommand("Shoot Amp", Commands.print("*******************************Shoot into Amp"));
        NamedCommands.registerCommand("Floor Intake",
                Commands.print("*******************************Activate Floor Intake"));
        NamedCommands.registerCommand("Go to Amp Position",
                Commands.print("*******************************Go to Amp Position for the Elevator"));
        NamedCommands.registerCommand("Spin Up Intake Flywheel",
                Commands.print("*******************************Go to Spin Up Intake Flywheel"));
        NamedCommands.registerCommand("Go to Speaker Position",
                Commands.print("*******************************Go to Speaker Position for the Elevator"));

        autoChooser = AutoBuilder.buildAutoChooser();
        autoChooser.setDefaultOption("Right-Side Straight-Line Auto",
                new PathPlannerAuto("Right-Side Straight-Line Auto"));
        Shuffleboard.getTab("Auto").add("Path Name", autoChooser);

        configureBindings();

    //     if (DriverStation.getJoystickName(ElectronicsIDs.DriverControllerPort).equals("Logitech Extreme 3D")) {
    //         driveSub.setDefaultCommand(
    //         // The left stick controls translation of the robot.
    //         // Turning is controlled by the X axis of the right stick.
    //         new DriveRobot(
    //                 driveSub, 
    //                 driverController, 
    //                 LogitechDAConstants.LeftStickX, LogitechDAConstants.LeftStickY, LogitechDAConstants.RightStickX, 
    //                 true));
    //         driveRobotWithAlignCmd = new DriveRobotWithAlign(
    //             driveSub,
    //             driverController,
    //             LogitechDAConstants.LeftStickX, LogitechDAConstants.LeftStickY, LogitechDAConstants.RightStickX,
    //             true,
    //             visionSub,
    //             autoAlignButton);
    // } else {
    //         driveSub.setDefaultCommand(
    //         new DriveRobot(
    //                 driveSub, 
    //                 driverController, 
    //                 RadioMasterConstants.LeftGimbalX, RadioMasterConstants.LeftGimbalY, RadioMasterConstants.RightGimbalX, 
    //                 true));
    //         driveRobotWithAlignCmd = new DriveRobotWithAlign(
    //             driveSub,
    //             driverController,
    //             RadioMasterConstants.LeftGimbalX, RadioMasterConstants.LeftGimbalY, RadioMasterConstants.RightGimbalX, 
    //             true,
    //             visionSub,
    //             autoAlignButton);
    // }


        DriveRobotWithAlign driveRobotWithAlignCmd = new DriveRobotWithAlign(
                driveSub,
                driverController,
                LogitechExtreme3DConstants.AxisX, LogitechExtreme3DConstants.AxisY, LogitechExtreme3DConstants.AxisZRotate,
                true,
                visionSub,
                autoAlignButton);

        autoAlignButton.whileTrue(driveRobotWithAlignCmd);

        // autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be
        // `Commands.none()`
        // SmartDashboard.putData("Auto Mode", autoChooser);
        autoChooser.setDefaultOption("Right-Side Straight-Line Auto", new PathPlannerAuto("Right-Side Straight-Line Auto"));
        Shuffleboard.getTab("Match").add("Path Name", autoChooser);

        configureBindings();

        driveSub.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new DriveRobot(
                driveSub,
                () -> driverController.getRawAxis(LogitechExtreme3DConstants.AxisX),
                () -> driverController.getRawAxis(LogitechExtreme3DConstants.AxisY),
                () -> -driverController.getRawAxis(LogitechExtreme3DConstants.AxisZRotate),
                true));
    }

    private void configureBindings() {

        driverController = new Joystick(ElectronicsIDs.DriverControllerPort);
        operatorController = new Joystick(ElectronicsIDs.OperatorControllerPort);

        /***************** AUTO ALIGN *****************/

        autoAlignButton = new JoystickButton(operatorController, XboxControllerConstants.LeftTrigger);
        autoAlignButton = new JoystickButton(driverController, LogitechExtreme3DConstants.ButtonStick);
        // autoAlignButton.onTrue(autoAlignCmd).onFalse();

        /******************** SET SHOOTER POSITION ********************/

        moveToSpeakerButton = new JoystickButton(operatorController, XboxControllerConstants.RightBumper); // middle
        moveToSpeakerButton.onTrue(setShooterPosSpeakerCmd);

        moveToAmpButton = new JoystickButton(operatorController, XboxControllerConstants.LeftBumper); // top
        moveToAmpButton.onTrue(setShooterPosAmpCmd);

        moveToSourceButton = new JoystickButton(operatorController, XboxControllerConstants.ButtonY); // claw
        moveToSourceButton.onTrue(setShooterPosSourceIntakeCmd);

        moveToFloorButton = new JoystickButton(operatorController, XboxControllerConstants.RightStick); // station
        moveToFloorButton.onTrue(setShooterPosFloorIntakeCmd);

        moveToTrapButton = new JoystickButton(operatorController, XboxControllerConstants.ButtonB); // no button on
                                                                                                    // mantis controller
        moveToTrapButton = new JoystickButton(operatorController, XboxControllerConstants.ButtonB); // no button on mantis controller
        moveToTrapButton.onTrue(setShooterPosTrapCmd);

        /******************** SHOOTER ********************/

        shootIntakeButton = new JoystickButton(operatorController, XboxControllerConstants.ButtonA); // home
        shootIntakeButton.onTrue(startShooterIntakeCmd).onFalse(stopShooterIntakeCmd);

        /***************** FLOOR INTAKE *****************/

        // toggleFloorIntakeButton = new JoystickButton(operatorController,
        // XboxControllerConstants.ButtonX); // floor
        // toggleFloorIntakeButton.onTrue(startIntakeCmd).onFalse(stopIntakeCmd);

        /******************** CLIMB ********************/

        // climbButton = new JoystickButton(operatorController,
        // XboxControllerConstants.ButtonX); // no button on mantis controller
        // climbButton.onTrue(climbCmd);

        /********************* LED BINDINGS ************************************* */

        LEDHumanSourceButton = new JoystickButton(operatorController, XboxControllerConstants.LeftBumper);
        LEDHumanSourceButton.onTrue(new InstantCommand(() -> underglowSub.LEDHumanSource = true))
                .onFalse(new InstantCommand(() -> underglowSub.LEDHumanSource = false));

        LEDHumanFloorButton = new JoystickButton(operatorController, XboxControllerConstants.ButtonX);
        LEDHumanFloorButton.onTrue(new InstantCommand(() -> underglowSub.LEDHumanFloor = true))
                .onFalse(new InstantCommand(() -> underglowSub.LEDHumanFloor = false));
        shootIntakeButton = new JoystickButton(operatorController, XboxControllerConstants.ButtonA); // home
        shootIntakeButton = new JoystickButton(driverController, LogitechExtreme3DConstants.Trigger);
        shootIntakeButton.onTrue(startShooterIntakeCmd).onFalse(stopShooterIntakeCmd);

        /******************** CLIMB ********************/

        // prepareToClimbButton = new JoystickButton(operatorController,
        // XboxControllerConstants.ButtonX); // no button on mantis controller. CHANGE
        // button binding
        // prepareToClimbButton.onTrue(prepareToClimbCmd);

        // climbButton = new JoystickButton(operatorController,
        // XboxControllerConstants.ButtonX); // no button on mantis controller. CHANGE
        // button binding
        // climbButton.onTrue(climbCmd);

        /******************** MAX VELOCITY SWITCHER ********************/
        /***************** AUTO ALIGN ******************/
        autoAlignButton = new JoystickButton(operatorController, XboxControllerConstants.ButtonX);
    }

    private void configurePathPlannerLogging() {

        PathPlannerLogging.setLogCurrentPoseCallback(
            (currentPose) -> {
                Logger.recordOutput("Odometry/CurrentPose", currentPose);
                });
        PathPlannerLogging.setLogActivePathCallback(
                (activePath) -> {
                    Logger.recordOutput(
                            "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
                });
        PathPlannerLogging.setLogTargetPoseCallback(
                (targetPose) -> {
                    Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
                });
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
        // return new PathPlannerAuto("Score in Amp Speaker Speaker V1 (SASS)");
    }
}