// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SerialPort;

public class Constants {

    public static final double SecondsPerMinute = 60;
    public static final double jKgMetersSquared = 0.0005;
    public static final double Neo550MaxRPM = 11000;
    public static final double NeoMaxRPM = 5820;
    public static final double Falcon500MaxRPM = 6300;
    public static final double KrakenX60MaxRPM = 6000;

    public static final double TalonFXUnitsPerRotation = 2048;
    public static final double CANCoderUnitsPerRotation = 4096;

    // public static final double toleranceLimit = 0.05;
    // public static final double LowerToleranceLimit = 1 - toleranceLimit;
    // public static final double UpperToleranceLimit = 1 + toleranceLimit;

    /***************************************************************************/
    /***************************************************************************/
    /***************************************************************************/

    public static final class ElectronicsIDs {

        public static final int DriverControllerPort = 1;
        public static final int OperatorControllerPort = 2;

        /***************************** DRIVE *****************************/

        // Encoder = 1{locationOnBot}
        public static final int FrontLeftTurnEncoderID = 11;
        public static final int FrontRightTurnEncoderID = 12;
        public static final int BackLeftTurnEncoderID = 13;
        public static final int BackRightTurnEncoderID = 14;

        // DriveMotorID = 2{locationOnBot} // Base
        public static final int FrontLeftDriveMotorID = 21;
        public static final int FrontRightDriveMotorID = 22;
        public static final int BackLeftDriveMotorID = 23;
        public static final int BackRightDriveMotorID = 24;

        // TurnMotorID = 3{locationOnBot} // Side
        public static final int FrontLeftTurnMotorID = 31;
        public static final int FrontRightTurnMotorID = 32;
        public static final int BackLeftTurnMotorID = 33;
        public static final int BackRightTurnMotorID = 34;

        /***************************** SHOOTER *****************************/

        // ShooterMotorID = 4{locationOnBot}
        public static final int LowerShooterMotorID = 41;
        public static final int UpperShooterMotorID = 42;
        public static final int AngleMotorID = 43;
        public static final int IndexMotorID = 44;
        public static final int BreakBeamID = 4;

        /***************************** FLOOR INTAKE *****************************/

        // FloorMotorID = 5{locationOnBot}
        public static final int FloorMotorID = 51;

        /***************************** ELEVATOR *****************************/

        public static final int LeftElevatorMotorID = 61;
        public static final int RightElevatorMotorID = 62;
        public static final int AngleEncoderID = 63;

        public static final int BottomHallEffectID = 6;
        // public static final int TopHallEffectID = 7;
    }

    /***************************************************************************/
    /***************************************************************************/
    /***************************************************************************/

    public static final class DriveConstants {

        public static final boolean GyroReversed = false;

        public static final double TrackWidth = Units.inchesToMeters(22); // Distance between left and right wheels
        public static final double WheelBase = Units.inchesToMeters(20); // Distance between front and back wheels
        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                new Translation2d(WheelBase / 2, TrackWidth / 2), // front left
                new Translation2d(WheelBase / 2, -TrackWidth / 2), // front right
                new Translation2d(-WheelBase / 2, TrackWidth / 2), // back left
                new Translation2d(-WheelBase / 2, -TrackWidth / 2) // back right
        );

        public static final double WheelRadius = Units.inchesToMeters(2.0);
        public static final double WheelCircumference = 2.0 * WheelRadius * Math.PI;
        public static final double GearRatio = 6.12;

        public static final double VelocityConversionFactor = WheelCircumference / Constants.SecondsPerMinute / GearRatio;

        public static final double MaxAngularRadiansPerSecond = Math.PI; // 1/2 rotation per second
        public static final double PhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI; // CHANGE

        public static final double MaxAcceleration = Units.feetToMeters(36.24); // m/sec^2 from Mr. K's spreadsheet
        public static final double MaxDriveableVelocity = 3.6; // m/s
        public static final double MaxModuleSpeed = NeoMaxRPM * VelocityConversionFactor;
 
        public static final double FrontLeftMagnetOffsetInRadians = 1.5171039327979088;
        public static final double FrontRightMagnetOffsetInRadians = 1.7456666082143784;
        public static final double BackLeftMagnetOffsetInRadians = -2.7626938149333;
        public static final double BackRightMagnetOffsetInRadians = -2.305568464100361;

        public static final double TimestepDurationInSeconds = 0.02;
        public static final Translation2d flModuleOffset = new Translation2d(0.4, 0.4);
        public static final Translation2d frModuleOffset = new Translation2d(0.4, -0.4);
        public static final Translation2d blModuleOffset = new Translation2d(-0.4, 0.4);
        public static final Translation2d brModuleOffset = new Translation2d(-0.4, -0.4);

        public static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
                new PIDConstants(5.0, 0, 0), // Translation constants
                new PIDConstants(5.0, 0, 0), // Rotation constants
                MaxModuleSpeed,
                flModuleOffset.getNorm(), // Drive base radius (distance from center to furthest module)
                new ReplanningConfig());

        /* DRIVE ENCODER */
        public static final double DriveKP = 0.04; // REQUIRES TUNING
        public static final double DriveKI = 0.0015;
        public static final double DriveKD = 0;
        public static final double DriveIZone = 0.15;
        public static final double DriveFF = 1.0 / MaxModuleSpeed;

        public static final double AutoAlignKP = 0.1; //CHANGE
        public static final double AutoAlignKI = 0.0015;
        public static final double AutoAlignKD = 0;

        /* TURN ENCODER */
        public static final int CANCoderResolution = 4096;
        public static final double PositionConversionFactor = WheelCircumference / GearRatio;
        public static final double TurnKP = 1; // Need to change
        public static final double TurnKI = 0;
        public static final double TurnKD = 0;

        public static final double ModuleMaxAngularVelocity = 3.0 * 2.0 * Math.PI; // #revolutions * radians per
                                                                                   // revolution (rad/sec)
        public static final double ModuleMaxAngularAcceleration = 12 * Math.PI; // radians per second squared

        public static final int StallLimit = 40;
        public static final int FreeLimit = 40;
    }

    public static final class AutoConstants {
        public static final double MaxSpeedMetersPerSecond = DriveConstants.MaxModuleSpeed / 4; // CHANGE
        public static final double MaxAngularSpeedRadiansPerSecond = DriveConstants.PhysicalMaxAngularSpeedRadiansPerSecond
                / 10; // Default is 540 degress
        // public static final double MaxAccelerationMetersPerSecondSquared = 3;
        public static final double MaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4; // default: 720 deg
    }

    /***************************************************************************/
    /***************************************************************************/
    /***************************************************************************/
    public static final class UnderGlowConstants {
        public static final SerialPort.Port Port = SerialPort.Port.kUSB1;
        public static final int BlueAlliance = 0; // 0b00000000
        public static final int RedAlliance = 1; // 0b00000001
        public static final int Enabled = 2; // 0b00000010
        public static final int NoteLoaded = 4; // 0b00000100
        public static final int Shooting = 8; // 0b00001000
        public static final int Auto = 16; // 0b00010000
        public static final int RobotFloorSource = 32; // 0b00100000
        public static final int Climbing = 64; // 0b01000000
        public static final int RobotSource = 128; // 0b10000000
    }

    public static final class ShooterConstants {
        public static final double FlywheelGearRatio = 1.5; // 36 gears on motor, 24 on rollers --> 1.5:1 (as of 2/15)
        public static final double IndexGearRatio = 1; // 1:1 ratio on Index per WK as of 2/15

        
        public static final double VelocityTolerance = 0.05; // CHANGE

        public static final double SpeakerRPM = 4000; // CHANGE
        public static final double AmpRPM = 2000; // CHANGE
        public static final double IntakeRPM = -2000; // CHANGE
        public static final double TrapRPM = 2000; // CHANGE

        public static final double ShooterKP = 0.5; // An error of 1 rotation/sec results in 2V output
        public static final double ShooterKI = 0; 
        public static final double ShooterKD = 0; // CHANGE ?
        public static final double ShooterFF = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second

        public static final double IndexRPM = 180; // CHANGE

        public static final double IndexKP = 0.5; // CHANGE
        public static final double IndexKI = 0; // CHANGE
        public static final double IndexKD = 0; // CHANGE
        public static final double IndexIZone = 0; // CHANGE
        public static final double IndexFF = 0.12; // CHANGE

        public static final double SupplyCurrentLimit = 30;
    }

    /***************************************************************************/
    /***************************************************************************/
    /***************************************************************************/

    public static final class ShooterMountConstants {

        public static final double AngleTolerance = 1.5; // Degrees - CHANGE
        public static final double HeightTolerance = 2; // Inches - CHANGE

        public static final double ElevatorGearRatio = 15;
        public static final double ShooterAngleGearRatio = 46.67; // From K's spreadsheet
        public static final double ElevatorSprocketDiameter = Units.inchesToMeters(2.36);
        public static final double ElevatorSprocketCircumference = ElevatorSprocketDiameter * Math.PI;

        public static final double ShooterAngleMaxSpeed = (Falcon500MaxRPM / 60 * 360) / ElevatorGearRatio; // deg/sec
        public static final double ElevatorMaxSpeed = (Falcon500MaxRPM / 60 / ElevatorGearRatio)
                * ElevatorSprocketCircumference; // m/s
        public static final double RotationsPerElevatorInch = ElevatorGearRatio / Units.metersToInches(ElevatorSprocketCircumference);

        public static final int IndexMotorCurrentLimit = 30; // CHANGE
        public static final double MaxHeightInches = 10;

        public static final double SpeakerAngle = 110; // CHANGE
        public static final double SpeakerHeight = 0; // Resting position

        public static final double AmpAngle = 10; // CHANGE
        public static final double AmpHeight = 16; // CHANGE

        public static final double SourceIntakeAngle = 60; // CHANGE
        public static final double SourceIntakeHeight = 15; // CHANGE

        public static final double FloorIntakeAngle = 0; // Resting position
        public static final double FloorIntakeHeight = 0; // CHANGE

        public static final double TrapAngle = 90; // CHANGE
        public static final double TrapHeight = 10; // CHANGE

        public static final double ClimbHeight = 10; // CHANGE
        public static final double MinHeight = 0; // CHANGE

        public static final double AngleKP = 0.5;
        public static final double AngleKI = 0;
        public static final double AngleKD = 0.4;
        public static final double AngleIZone = 0;
        public static final double AngleFF = 0.11;

        public static final double AngleMMCruiseVel = .25; //1.5; // CHANGE - Target cruise velocity of 1.5 rps
        public static final double AngleMMAcceleration = 1;//3; // CHANGE - Target acceleration of 3 rps/s (0.5 seconds)
        public static final double AngleMMJerk = 10; //30; // CHANGE - Target jerk of 30 rps/s/s (0.1 seconds)

        public static final double ElevatorKP = 0.5;
        public static final double ElevatorKI = 0;
        public static final double ElevatorKD = 0.4;
        public static final double ElevatorIZone = 0;
        public static final double ElevatorFF = 0.12;

        public static final double ElevatorMMCruiseVel = 40; // CHANGE - Target cruise velocity of 80 rps
        public static final double ElevatorMMAcceleration = 80; // CHANGE - Target acceleration of 160 rps/s (0.5 seconds)
        public static final double ElevatorMMJerk = 800; // CHANGE - Target jerk of 1600 rps/s/s (0.1 seconds)
    }

    /***************************************************************************/
    /***************************************************************************/
    /***************************************************************************/

    public static final class FloorIntakeConstants {
        /* PID CONTROLLER */
        public static final double KP = 0.5; // CHANGE
        public static final double KI = 0.1; // CHANGE
        public static final double KD = 0.1; // CHANGE
        public static final double IZone = 0.1; // CHANGE
        public static final double FF = 0.12; // CHANGE

        public static final int SupplyCurrentLimit = 20;
		
        public static final double MotorRPM = KrakenX60MaxRPM * 0.4;
        public static final double VelocityTolerance = 0.05; // CHANGE
    }

    /***************************************************************************/
    /***************************************************************************/
    /***************************************************************************/

    public static final class VisionConstants {
        public static final int CameraLightID = 0; // Need to change
        public static final String kPoseCameraName = "Global_Shutter_Camera";
        public static final String kTargetCameraName = "Arducam_OV9281_USB_Camera";

        public static final PoseStrategy kPrimaryVisionStrategy = PoseStrategy.CLOSEST_TO_REFERENCE_POSE;
        public static final PoseStrategy kFallbackVisionStrategy = PoseStrategy.LOWEST_AMBIGUITY;
        // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
        public static final Transform3d kRobotToPoseCam =
                new Transform3d(new Translation3d(0.0, 0.0, 1.0), new Rotation3d(0, 0, 0));
        public static final Transform3d kRobotToTargetCam =
                new Transform3d(new Translation3d(0.0, 0.0, 1.0), new Rotation3d(0, 0, 0));

        // // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout kFieldTagLayout =
                AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

        // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    }

    /***************************************************************************/
    /***************************************************************************/
    /***************************************************************************/

    public final class LogitechDAConstants {
        public static final int LeftStickX = 0; // LDA = Logitech Dual Action
        public static final int LeftStickY = 1;
        public static final int RightStickX = 2;
        public static final int RightStickY = 3;
        public static final int LeftTrigger = 7; // Speaker
        public static final int RightTrigger = 8; // Amp
        public static final int ButtonA = 2; // Move Source
        public static final int ButtonB = 3; // Trapdoor
        public static final int ButtonX = 1; // Move Floor
        public static final int ButtonY = 4; // Move Trap
        public static final int LeftBumper = 5; // Floor Intake
        public static final int RightBumper = 6; // Source Intake
        public static final int BackButton = 9; // Climb
        public static final int StartButton = 10;
        public static final int LeftStick = 11; // Move Speaker
        public static final int RightStick = 12; // Move Amp
        public static final double ForwardAxisAttenuation = -0.5;
        public static final double LateralAxisAttenuation = 0.5;
        public static final double YawAxisAttenuation = 0.5;
    }

    public final class LogitechExtreme3DConstants {
        public static final int AxisX = 0; 
        public static final int AxisY = 1;
        public static final int AxisZRotate = 2; 
        public static final int Slider = 3; 
        public static final int Trigger = 1; 
        public static final int ButtonStick = 2; 
        public static final int Button3 = 3; 
        public static final int Button4 = 4; 
        public static final int Button5 = 5; 
        public static final int Button6 = 6;
        public static final int Button7 = 7; 
        public static final int Button8 = 8;
        public static final int Button9 = 9; 
        public static final int Button10 = 10;
        public static final int Button11 = 11;
        public static final int Button12 = 12;
        public static final double ForwardAxisAttenuation = -0.5;
        public static final double LateralAxisAttenuation = 0.5;
        public static final double YawAxisAttenuation = 0.5;
    }

    public final class RadioMasterConstants {
        public static final int LeftGimbalX = 0;
        public static final int LeftGimbalY = 1;
        public static final int RightGimbalX = 3;
        public static final int RightGimbalY = 2;
        public static final int SliderF = 5;
        public static final int SliderE = 4;
        public static final int SliderC = 6;
        public static final int ButtonD = 2;
        public static final int ButtonA = 1;
        public static final double FowardAxisAttenuation = 1.0;
        public static final double LateralAxisAttenuation = 1.0;
        public static final double YawAxisAttenuation = 0.6;
    }

    public final class XboxControllerConstants {
        public static final int LeftStickX = 0;
        public static final int LeftStickY = 1;
        public static final int LeftTrigger = 2;
        public static final int RightTrigger = 4;
        public static final int RightStickX = 4;
        public static final int RightStickY = 5;
        // Angle Trap?

        // need to CHANGE these comments b/c they're not right
        public static final int ButtonA = 1; // Shoot Trap
        public static final int ButtonB = 2; // Climb
        public static final int ButtonX = 3; // Auto Align
        public static final int ButtonY = 4; // Angle Speaker
        public static final int LeftBumper = 5; // Move to Amp
        public static final int RightBumper = 6; // Shoot Speaker
        // public static final int BackButton = 7;
        public static final int StartButton = 8; // Angle Amp
        public static final int LeftStick = 9; // Angle Source
        public static final int RightStick = 10; // Source Intake
        public static final int WindowButton = 7; // Angle Floor

        public static final double ForwardAxisAttenuation = -0.5;
        public static final double LateralAxisAttenuation = 0.5;
        public static final double YawAxisAttenuation = 0.5;
    }
}
