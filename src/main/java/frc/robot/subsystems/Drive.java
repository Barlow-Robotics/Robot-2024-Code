// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElectronicsIDs;
import org.littletonrobotics.junction.Logger;
import java.lang.Math;
import frc.robot.Constants;

import com.kauailabs.navx.frc.AHRS;


public class Drive extends SubsystemBase {

    private final SwerveModule frontLeft = new SwerveModule(
            "frontLeft",
            ElectronicsIDs.FrontLeftDriveMotorID,
            ElectronicsIDs.FrontLeftTurnMotorID,
            ElectronicsIDs.FrontLeftTurnEncoderID,
            Math.toDegrees(DriveConstants.FrontLeftMagnetOffsetInRadians) / 360.0, 
            false
    );

    private final SwerveModule frontRight = new SwerveModule(
            "frontRight",
            ElectronicsIDs.FrontRightDriveMotorID,
            ElectronicsIDs.FrontRightTurnMotorID,
            ElectronicsIDs.FrontRightTurnEncoderID,
            Math.toDegrees(DriveConstants.FrontRightMagnetOffsetInRadians) / 360.0, 
            true
    );

    private final SwerveModule backLeft = new SwerveModule(
            "backLeft",
            ElectronicsIDs.BackLeftDriveMotorID,
            ElectronicsIDs.BackLeftTurnMotorID,
            ElectronicsIDs.BackLeftTurnEncoderID,
            Math.toDegrees(DriveConstants.BackLeftMagnetOffsetInRadians) / 360.0, 
            false
    );

    private final SwerveModule backRight = new SwerveModule(
            "backRight",
            ElectronicsIDs.BackRightDriveMotorID,
            ElectronicsIDs.BackRightTurnMotorID,
            ElectronicsIDs.BackRightTurnEncoderID,
            Math.toDegrees(DriveConstants.BackRightMagnetOffsetInRadians) / 360.0,
            true
    );

    private AHRS navX;

    private final SwerveDriveOdometry odometry;

    private SwerveModulePosition[] previousPositions = new SwerveModulePosition[4] ;


    public Drive() {
        navX = new AHRS(Port.kMXP);
        navX.reset();

        odometry = new SwerveDriveOdometry(
            DriveConstants.kinematics,
            navX.getRotation2d(),
            new SwerveModulePosition[] {
                    frontLeft.getPosition(),
                    frontRight.getPosition(),
                    backLeft.getPosition(),
                    backRight.getPosition()
            });
    }

    @Override
    public void periodic() {
        odometry.update(
                navX.getRotation2d(),
                new SwerveModulePosition[] {
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        backLeft.getPosition(),
                        backRight.getPosition()
                });

        Logger.recordOutput("Pose", odometry.getPoseMeters());
        SwerveModuleState[] swerveModuleActualStates = new SwerveModuleState[] {frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState()};
        Logger.recordOutput("SwerveStates/ActualStates", swerveModuleActualStates);
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }
    
    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(
                navX.getRotation2d(),
                new SwerveModulePosition[] {
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        backLeft.getPosition(),
                        backRight.getPosition()
                },
                pose);
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

        var swerveModuleDesiredStates = DriveConstants.kinematics.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot,
                                navX.getRotation2d())
                        : new ChassisSpeeds(xSpeed, ySpeed, rot));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleDesiredStates, DriveConstants.MaxDriveableVelocity);
        frontLeft.setDesiredState(swerveModuleDesiredStates[0]);
        frontRight.setDesiredState(swerveModuleDesiredStates[1]);
        backLeft.setDesiredState(swerveModuleDesiredStates[2]);
        backRight.setDesiredState(swerveModuleDesiredStates[3]);

        Logger.recordOutput("SwerveStates/DesiredStates", swerveModuleDesiredStates);
    }

    public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
        driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPose().getRotation()));
    }

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, DriveConstants.TimestepDurationInSeconds);
        SwerveModuleState[] targetStates = DriveConstants.kinematics.toSwerveModuleStates(targetSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, Constants.DriveConstants.MaxDriveableVelocity);
        frontLeft.setDesiredState(targetStates[0]);
        frontRight.setDesiredState(targetStates[1]);
        backLeft.setDesiredState(targetStates[2]);
        backRight.setDesiredState(targetStates[3]);
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.DriveConstants.MaxDriveableVelocity);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    public void resetEncoders() {
        frontLeft.resetEncoders();
        backLeft.resetEncoders();
        frontRight.resetEncoders();
        backRight.resetEncoders();
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void zeroHeading() {
        navX.reset();
    }

    public double getHeading() {
        return navX.getRotation2d().getDegrees();
    }

    public double getTurnRate() {
        return navX.getRate() * (DriveConstants.GyroReversed ? -1.0 : 1.0); // degrees per second
    }

    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()
        };
      }

    public ChassisSpeeds getSpeeds() {
        return DriveConstants.kinematics.toChassisSpeeds(getModuleStates());
    }

    public void simulationInit() {
        frontLeft.simulationInit();
        frontRight.simulationInit();
        backLeft.simulationInit();
        backRight.simulationInit();

        previousPositions[0] = frontLeft.getPosition();
        previousPositions[1] = frontRight.getPosition();
        previousPositions[2] = backLeft.getPosition();
        previousPositions[3] = backRight.getPosition();
    }

    @Override
    public void simulationPeriodic() {
        var modulePositions = new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
            } ;
    
        var moduleDeltas = new SwerveModulePosition[modulePositions.length];
        for (int index = 0; index < modulePositions.length; index++) {
                var current = modulePositions[index];
                var previous = previousPositions[index];

                moduleDeltas[index] = new SwerveModulePosition(current.distanceMeters - previous.distanceMeters, current.angle);
                previous.distanceMeters = current.distanceMeters;
        }
        var twist = DriveConstants.kinematics.toTwist2d(moduleDeltas);

        int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
        SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
        angle.set(navX.getAngle() - Units.radiansToDegrees(twist.dtheta));
    }    
    
}
