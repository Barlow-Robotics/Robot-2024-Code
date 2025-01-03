// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;

import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SignalsConfig;

public class SwerveModule {

    private final SparkMax driveMotor;
    private final RelativeEncoder driveEncoder;

    private final SparkMax turnMotor;
    private final CANcoder turnEncoder;
    public final ProfiledPIDController turnPIDController;
    private final SimpleMotorFeedforward TurnFF = new SimpleMotorFeedforward(0, 0.4); // Need to change these #'s
    SimDeviceSim driveMotorSim;
    SimDeviceSim turnMotorSim;
    private String swerveName;
    public final SparkClosedLoopController drivePIDController;


    public SwerveModule(
            String name,
            int driveMotorID,
            int turningMotorID,
            int turnEncoderID,
            double magnetOffset,
            boolean reversed) {
        turnMotor = new SparkMax(turningMotorID, SparkLowLevel.MotorType.kBrushless);

        SparkMaxConfig driveSparkMaxConfig = new SparkMaxConfig();
        SparkMaxConfig turnSparkMaxConfig = new SparkMaxConfig();

        /* Set up drive motor and encoder */
        swerveName = name;
        driveMotor = new SparkMax(driveMotorID, SparkLowLevel.MotorType.kBrushless);

        drivePIDController = driveMotor.getClosedLoopController();
        // driveMotor.restoreFactoryDefaults();
        driveSparkMaxConfig
            .inverted(reversed)
            .idleMode(IdleMode.kBrake);
        driveMotorSim = new SimDeviceSim("SPARK MAX [" + driveMotor.getDeviceId() + "]");
        turnMotorSim = new SimDeviceSim("SPARK MAX [" + turnMotor.getDeviceId() + "]");

        
        driveEncoder = driveMotor.getEncoder();
        resetEncoders();


        double localPositionConversionFactor = DriveConstants.PositionConversionFactor;
        if (RobotBase.isSimulation()) {
            localPositionConversionFactor *= 1000;
        }
        
        driveSparkMaxConfig.encoder
            .positionConversionFactor(localPositionConversionFactor)
            .velocityConversionFactor(DriveConstants.VelocityConversionFactor);

        // sparkMax
        /* Config drive motor PID */
        driveSparkMaxConfig.closedLoop
            .pidf(DriveConstants.DriveKP, DriveConstants.DriveKI, DriveConstants.DriveKD, DriveConstants.DriveFF)
            .iZone(DriveConstants.DriveIZone)
            .outputRange(-1, 1);        
        /* Set up turn motor and encoder */
        // turnMotor.restoreFactoryDefaults();
        turnSparkMaxConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake);

        turnEncoder = new CANcoder(turnEncoderID, "rio");
        var canCoderConfiguration = new CANcoderConfiguration();
        MagnetSensorConfigs magnetConfig = new MagnetSensorConfigs();
        magnetConfig.AbsoluteSensorDiscontinuityPoint = 0.5;
        if (!Robot.isSimulation()) {
            magnetConfig.MagnetOffset = magnetOffset;
        } else {
            magnetConfig.MagnetOffset = 0.0;
        }
        magnetConfig.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        canCoderConfiguration.MagnetSensor = magnetConfig;

        turnEncoder.getConfigurator().apply(canCoderConfiguration);
        if (Robot.isSimulation()) {
            turnEncoder.setPosition(0.0, 0.1);
        }

        // turnPIDController = new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(
        //         DriveConstants.ModuleMaxAngularVelocity, DriveConstants.ModuleMaxAngularAcceleration));

        turnPIDController = new ProfiledPIDController(3, 0, 0, new TrapezoidProfile.Constraints(
                DriveConstants.ModuleMaxAngularVelocity, DriveConstants.ModuleMaxAngularAcceleration));

                turnPIDController.enableContinuousInput(-Math.PI, Math.PI);

        // wpk test - current limitrrs result in slow module rotation. need to investigate further
        driveSparkMaxConfig.smartCurrentLimit(DriveConstants.StallLimit, DriveConstants.FreeLimit);
        turnSparkMaxConfig.smartCurrentLimit(DriveConstants.StallLimit, DriveConstants.FreeLimit);

        driveMotor.configure(driveSparkMaxConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        turnMotor.configure(turnSparkMaxConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        // driveMotor.smartCurrentLimit(DriveConstants.StallLimit, DriveConstants.FreeLimit);
        // turnMotor.setSmartCurrentLimit(DriveConstants.StallLimit, DriveConstants.FreeLimit);

    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                driveEncoder.getVelocity(),
                new Rotation2d(turnEncoder.getAbsolutePosition().getValueAsDouble() * 2.0* Math.PI));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                driveEncoder.getPosition(),
                new Rotation2d(turnEncoder.getAbsolutePosition().getValueAsDouble() * 2.0 * Math.PI));
    }

    public double getTurnCurrent() {
        return turnMotor.getOutputCurrent();
    } 

    public double getDriveCurrent() {
        return driveMotor.getOutputCurrent();
    } 

    public void setDesiredState(SwerveModuleState desiredState) {

        // Optimize the reference state to avoid spinning further than 90 degrees

        SwerveModuleState state = SwerveModuleState.optimize(desiredState,
                new Rotation2d(turnEncoder.getAbsolutePosition().getValueAsDouble()* 2.0 * Math.PI));

        Logger.recordOutput(swerveName + " Drive velocity", driveMotor.getEncoder().getVelocity());

        drivePIDController.setReference(state.speedMetersPerSecond, ControlType.kVelocity);

        final double turnOutput = turnPIDController.calculate(
                turnEncoder.getAbsolutePosition().getValueAsDouble() * 2.0 * Math.PI,
                state.angle.getRadians());
        final double turnFF = TurnFF.calculate(turnPIDController.getSetpoint().velocity);
        turnMotor.setVoltage(turnOutput + turnFF);

        if (RobotBase.isSimulation()) {
            CANcoderSimState encoderSim = turnEncoder.getSimState();
            encoderSim.setRawPosition(state.angle.getDegrees() / 360.0);
            Logger.recordOutput("CANCoder " + swerveName,
                    turnEncoder.getAbsolutePosition().getValueAsDouble());
        }
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
    }

    public void stop() {
        driveMotor.set(0);
        turnMotor.set(0);
    }

    public void simulationInit() {
        driveMotorSim.getDouble("Position").set(0.0);
        turnMotorSim.getDouble("Position").set(0.0);
    
        // You can also simulate velocity, temperature, etc.
        driveMotorSim.getDouble("Velocity").set(0.0);
        turnMotorSim.getDouble("Velocity").set(0.0);
    }
}
