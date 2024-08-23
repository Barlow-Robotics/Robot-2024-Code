// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.time.Instant;
import java.util.Optional;
import java.util.OptionalInt;
import com.choreo.lib.*;
// import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
// import com.pathplanner.lib.commands.PathPlannerAuto;
// import com.pathplanner.lib.controllers.PPHolonomicDriveController;
// import com.pathplanner.lib.util.PathPlannerLogging;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonUtils;

// import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
// import com.pathplanner.lib.commands.PathPlannerAuto;
// import com.pathplanner.lib.controllers.PPHolonomicDriveController;
// import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
// import com.pathplanner.lib.util.PIDConstants;
// import com.pathplanner.lib.util.PathPlannerLogging;
// import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElectronicsIDs;
import frc.robot.Constants.LogitechExtreme3DConstants;
// import frc.robot.Constants.ShooterMountConstants;
import frc.robot.Constants.XboxControllerConstants;
import frc.robot.autonomous.DynamicPathPlanner;
import frc.robot.commands.DriveRobotWithAprilTagAlign;
import frc.robot.commands.DriveRobotWithNoteAlign;
import frc.robot.commands.PivotToPoint;
import frc.robot.commands.ReverseFloorIntake;
// import frc.robot.commands.DriveRobotWithAlign;
import frc.robot.commands.SetShooterMountPosition;
import frc.robot.commands.StartClimbing;
import frc.robot.commands.StartShooterIntake;
import frc.robot.commands.StopShooterIntake;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.FloorIntake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterMount;
import frc.robot.subsystems.ShooterMount.ShooterMountState;
import frc.robot.subsystems.Underglow;
import frc.robot.subsystems.Vision;

public class ChoreoContainer {

    /* SUBSYSTEMS */

    public final Vision visionSub = new Vision();
    public final Drive driveSub = new Drive(visionSub);
    public final Shooter shooterSub = new Shooter();
    public final ShooterMount shooterMountSub = new ShooterMount(visionSub, driveSub);
    public final FloorIntake floorIntakeSub = new FloorIntake();
    public final Underglow underglowSub = new Underglow();

    /* COMMANDS */

    private final SetShooterMountPosition setShooterPosSpeakerCmd = new SetShooterMountPosition(shooterMountSub,
            ShooterMountState.Speaker);
    private final SetShooterMountPosition setShooterPosAmpCmd = new SetShooterMountPosition(shooterMountSub,
            ShooterMountState.Amp);
    private final SetShooterMountPosition setShooterPosSourceIntakeCmd = new SetShooterMountPosition(shooterMountSub,
            ShooterMountState.SourceIntake);
    private final SetShooterMountPosition setShooterPosFloorCmd = new SetShooterMountPosition(shooterMountSub,
            ShooterMountState.FloorIntake);
    private final SetShooterMountPosition setShooterPosFerryCmd = new SetShooterMountPosition(shooterMountSub,
            ShooterMountState.Ferry);

    private final StartClimbing climbCmd = new StartClimbing(shooterMountSub);
    // private final SetShooterMountPosition retryClimbCmd = new
    // SetShooterMountPosition(shooterMountSub,
    // ShooterMountState.Preclimb);
    // private final SetShooterMountPosition climbAbortCmd = new
    // 2SetShooterMountPosition(shooterMountSub,
    // ShooterMountState.ClimbAbort);

    private final StartShooterIntake startShooterIntakeCmd = new StartShooterIntake(shooterSub, floorIntakeSub,
            shooterMountSub);
    private final StopShooterIntake stopShooterIntakeCmd = new StopShooterIntake(shooterSub, floorIntakeSub);
    private final ReverseFloorIntake reverseFloorIntakeCmd = new ReverseFloorIntake(floorIntakeSub);


    private final PivotToPoint piviotToSpeakerCommand = new PivotToPoint(new Pose2d(0.44, 5.55, new Rotation2d(0)), driveSub);
    private final PivotToPoint piviotToCenterNoteCommand = new PivotToPoint(new Pose2d(2.90, 5.55, new Rotation2d(0)), driveSub);
    private final PivotToPoint piviotToAmpNoteCommand = new PivotToPoint(new Pose2d(2.90, 7, new Rotation2d(0)), driveSub);
    private final PivotToPoint piviotToStageNoteCommand = new PivotToPoint(new Pose2d(2.90, 4.10, new Rotation2d(0)), driveSub);


    private final PivotToPoint piviotToSpeakerCommandRED = new PivotToPoint(new Pose2d(16.1, 5.55, new Rotation2d(0)), driveSub);
    private final PivotToPoint piviotToCenterNoteCommandRED = new PivotToPoint(new Pose2d(13.66, 5.55, new Rotation2d(0)), driveSub);
    private final PivotToPoint piviotToAmpNoteCommandRED = new PivotToPoint(new Pose2d(13.66, 7, new Rotation2d(0)), driveSub);
    private final PivotToPoint piviotToStageNoteCommandRED = new PivotToPoint(new Pose2d(13.66, 4.1, new Rotation2d(0)), driveSub);


    /* CONTROLLERS */

    private static Joystick driverController;
    private static Joystick operatorController;

    /* BUTTONS */

    private Trigger moveToSpeakerButton; // button x
    private Trigger moveToAmpButton; // button y
    private Trigger moveToSourceButton; // left stick
    private Trigger moveToFloorButton; // left bumper
    private Trigger moveToFerryButton; // hamburger

    private Trigger climbButton; // button a
    private Trigger piviotToPoint;
    // private Trigger climbAbortButton; // right stick

    // private Trigger toggleLEDsButton; // hamburger
    // private Trigger LEDHumanSourceButton;
    // private Trigger LEDHumanFloorButton;

    private Trigger shootIntakeButton; // trigger
    private Trigger reverseFloorIntakeButton; // driver button 7

    private Trigger autoAlignButton; // driver button 11
    private Trigger restartGyroButton; // driver button 9

    private PIDController noteYawPID;
    private PIDController targetYawPID;
    ChoreoTrajectory traj;

    /* AUTO */

    private SendableChooser<Command> autoChooser;

    public ChoreoContainer() {
        noteYawPID = new PIDController(
                DriveConstants.YawOverrideAlignNoteKP,
                DriveConstants.YawOverrideAlignNoteKI,
                DriveConstants.YawOverrideAlignNoteKD);
        noteYawPID.setSetpoint(0.0);

        targetYawPID = new PIDController(
                DriveConstants.TargetYawOverrideAlignNoteKP,
                DriveConstants.TargetYawOverrideAlignNoteKI,
                DriveConstants.TargetYawOverrideAlignNoteKD);
        targetYawPID.setSetpoint(0.0);

        configureButtonBindings();
        driveSub.setDefaultCommand(
                // The left stick controls translation of the robot.
                // Turning is controlled by the X axis of the right stick.
                // new DriveRobot(
                // driveSub,
                // () -> -driverController.getRawAxis(LogitechExtreme3DConstants.AxisX),
                // () -> -driverController.getRawAxis(LogitechExtreme3DConstants.AxisY),
                // () -> -driverController.getRawAxis(LogitechExtreme3DConstants.AxisZRotate),
                // () -> -driverController.getRawAxis(LogitechExtreme3DConstants.Slider),
                // true));

                // new DriveRobotWithAprilTagAlign(
                // driveSub,
                // () -> -driverController.getRawAxis(LogitechExtreme3DConstants.AxisX),
                // () -> -driverController.getRawAxis(LogitechExtreme3DConstants.AxisY),
                // () -> -driverController.getRawAxis(LogitechExtreme3DConstants.AxisZRotate),
                // () -> -driverController.getRawAxis(LogitechExtreme3DConstants.Slider),
                // true,
                // visionSub,
                // // shooterMountSub,
                // () -> autoAlignButton.getAsBoolean()));

                new DriveRobotWithNoteAlign(
                        driveSub,
                        () -> -driverController.getRawAxis(LogitechExtreme3DConstants.AxisX),
                        () -> -driverController.getRawAxis(LogitechExtreme3DConstants.AxisY),
                        () -> -driverController.getRawAxis(LogitechExtreme3DConstants.AxisZRotate),
                        () -> -driverController.getRawAxis(LogitechExtreme3DConstants.Slider),
                        visionSub,
                        floorIntakeSub,
                        () -> autoAlignButton.getAsBoolean()));

    }

    private void configureButtonBindings() {
        driverController = new Joystick(ElectronicsIDs.DriverControllerPort);
        operatorController = new Joystick(ElectronicsIDs.OperatorControllerPort);

        /***************** DRIVE *****************/

        autoAlignButton = new JoystickButton(driverController, LogitechExtreme3DConstants.Button11);

        restartGyroButton = new JoystickButton(driverController, LogitechExtreme3DConstants.Button9);
        restartGyroButton.onTrue(new InstantCommand(() -> driveSub.zeroHeading()));

        /******************** SET SHOOTER MOUNT POSITION ********************/

        moveToSpeakerButton = new JoystickButton(operatorController, XboxControllerConstants.ButtonX);
        moveToSpeakerButton.onTrue(setShooterPosSpeakerCmd);

        moveToAmpButton = new JoystickButton(operatorController, XboxControllerConstants.ButtonY);
        moveToAmpButton.onTrue(setShooterPosAmpCmd);

        moveToSourceButton = new JoystickButton(operatorController, XboxControllerConstants.LeftStick);
        moveToSourceButton.onTrue(setShooterPosSourceIntakeCmd);

        moveToFloorButton = new JoystickButton(operatorController, XboxControllerConstants.LeftBumper);
        moveToFloorButton.onTrue(setShooterPosFloorCmd);

        moveToFerryButton = new JoystickButton(operatorController, XboxControllerConstants.HamburgerButton);
        moveToFerryButton.onTrue(setShooterPosFerryCmd);

        /******************** SHOOTER & INTAKE ********************/

        shootIntakeButton = new JoystickButton(driverController, LogitechExtreme3DConstants.Trigger);
        shootIntakeButton.onTrue(startShooterIntakeCmd).onFalse(stopShooterIntakeCmd);

        reverseFloorIntakeButton = new JoystickButton(driverController, LogitechExtreme3DConstants.Button7);
        reverseFloorIntakeButton.onTrue(reverseFloorIntakeCmd).onFalse(stopShooterIntakeCmd);

        /******************** CLIMB ********************/

        climbButton = new JoystickButton(operatorController, XboxControllerConstants.ButtonA);
        climbButton.onTrue(climbCmd);
    }

    public void configureChoreo() {
        // traj = Choreo.getTrajectory("New Path"); // (1)
        // (2)
        // traj.addEventMarker("PivotToSpeaker"); // This is a pseudo-code example. Replace with actual Choreo methods.
        // Choreo.registerCommand("PivotToSpeaker", piviotToSpeakerCommand);
        NamedCommands.registerCommand("StartShooterIntake", startShooterIntakeCmd);
        NamedCommands.registerCommand("StopShooterIntake", stopShooterIntakeCmd);
        NamedCommands.registerCommand("SetShooterMountPositionAmp", setShooterPosAmpCmd);
        NamedCommands.registerCommand("SetShooterMountPositionSpeaker", setShooterPosSpeakerCmd);
        NamedCommands.registerCommand("SetShooterMountPositionFloor", setShooterPosFloorCmd);

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if (alliance.get() == DriverStation.Alliance.Red) {
                NamedCommands.registerCommand("PivotToSpeaker", piviotToSpeakerCommandRED);
                NamedCommands.registerCommand("PivotToCenterNote", piviotToCenterNoteCommandRED);
                NamedCommands.registerCommand("PivotToAmpNote", piviotToAmpNoteCommandRED);
                NamedCommands.registerCommand("PivotToStageNote", piviotToStageNoteCommandRED);
            } else {
                NamedCommands.registerCommand("PivotToSpeaker", piviotToSpeakerCommand);
                NamedCommands.registerCommand("PivotToCenterNote", piviotToCenterNoteCommand);
                NamedCommands.registerCommand("PivotToAmpNote", piviotToAmpNoteCommand);
                NamedCommands.registerCommand("PivotToStageNote", piviotToStageNoteCommand);
            }
        }


        /* SMARTDASHBOARD */

        // autoChooser = AutoBuilder.buildAutoChooser(); // in order to remove autos, you must log into the roborio and
                                                      // delete them there
        // SmartDashboard.putData("Selected Auto", autoChooser);
        // autoChooser.setDefaultOption("BASIC", new PathPlannerAuto("BASIC"));
        // autoChooser.addOption("Routine A", new AutonomousRoutine("Routine A", visionSub));
        // autoChooser.addOption("Routine B", new AutonomousRoutine("Routine B", visionSub));
        // Shuffleboard.getTab("Match").add("Path Name", autoChooser);

        /* LOGGING */
        // PPHolonomicDriveController.setRotationTargetOverride(this::getRotationTargetOverride);

        // // var startingPoseTest =
        // // PathPlannerAuto.getStaringPoseFromAutoFile(autoChooser.getSelected()) ;

        // PathPlannerLogging.setLogCurrentPoseCallback(
        //         (currentPose) -> {
        //             Logger.recordOutput("Odometry/CurrentPose", currentPose);
        //         });
        // PathPlannerLogging.setLogActivePathCallback(
        //         (activePath) -> {
        //             Logger.recordOutput(
        //                     "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        //         });
        // PathPlannerLogging.setLogTargetPoseCallback(
        //         (targetPose) -> {
        //             Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        //         });
    }

    public Optional<Rotation2d> getRotationTargetOverride(){

        Optional<Rotation2d> result = Optional.empty() ;

        if (RobotState.isAutonomous()) {

            if ( shooterMountSub.getShooterMountState() == ShooterMountState.FloorIntake ) {
                if (visionSub.noteIsVisible() && Math.abs(visionSub.getNoteDistanceFromCenter()) < Constants.VisionConstants.NoteAlignPixelTolerance) {
                    double yaw = visionSub.getNoteDistanceFromCenter();
                    double rotDelta = noteYawPID.calculate(yaw);
                    result =  Optional.of( driveSub.getPose().getRotation().plus(new Rotation2d(rotDelta)) ) ;
                } else {
                    noteYawPID.reset();
                }
            } else if (shooterMountSub.getShooterMountState() == ShooterMountState.Speaker && !visionSub.allDetectedTargets.isEmpty()) {
                var bestTarget = visionSub.getBestTrackableTarget() ;
                if (bestTarget.isPresent()) {
                    var rotOffset = bestTarget.get().getYaw();
                    rotOffset = targetYawPID.calculate(rotOffset);
                    result = Optional.of( driveSub.getPose().getRotation().plus(new Rotation2d(rotOffset)) ) ;
                    result = Optional.empty() ;

                    // Logger.recordOutput("YawOverrideAlign/targetYaw", bestTarget.get().getYaw());
                    // Logger.recordOutput("YawOverrideAlign/proposed rot", result.get());
                            // Logger.recordOutput("YawOverrideAlign/rot offset", result.get());
                } else {

                }
                noteYawPID.reset();

            }
        }

        return result;
    }

    public Command getAutonomousCommand() {
        ChoreoTrajectory traj = Choreo.getTrajectory("Test1"); 

        Command swerveCommand = Choreo.choreoSwerveCommand(
            traj, // Choreo trajectory
            driveSub::getPose, // A function that returns the current field-relative pose of the robot
            new PIDController(5.0, 0.0, 0.0), // PIDController for field-relative X translation (matching your translation PID)
            new PIDController(5.0, 0.0, 0.0), // PIDController for field-relative Y translation
            new PIDController(5.0, 0.0, 0.5), // PIDController for rotation (matching your rotation PID)
            (ChassisSpeeds speeds) -> driveSub.driveRobotRelative(speeds),
            () -> {
                Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
                return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
            }, // Mirroring the path based on alliance (Red alliance mirrors)
            driveSub // The subsystem to require, typically your drive subsystem
        );



        return Commands.sequence(
            Commands.runOnce(() -> driveSub.resetOdometry(traj.getInitialPose())),
            swerveCommand,
            driveSub.run(() -> driveSub.drive(0, 0, 0, false))
        );

    }
}