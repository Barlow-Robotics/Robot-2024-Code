// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends LoggedRobot {
    private Command autonomousCommand;

    private RobotContainer robotContainer;

    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();

        Logger.recordMetadata("ProjectName", "WPI-Swerve-Prototype"); // Set a metadata value

        if (isReal()) {
            Logger.addDataReceiver(new WPILOGWriter("/media/sda2/")); // Log to a USB stick
            Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
            /*PowerDistribution pdp =*/ new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
        } else {
            Logger.addDataReceiver(new WPILOGWriter(""));
            Logger.addDataReceiver(new NT4Publisher());
        }

        Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.

        robotContainer.shooterSub.stopMotors();
        robotContainer.floorIntakeSub.stop();
    }

    @Override
    public void robotPeriodic() {
        SmartDashboard.putData(CommandScheduler.getInstance());
        SmartDashboard.putData(robotContainer.driveSub);
        SmartDashboard.putData(robotContainer.shooterSub);
        // SmartDashboard.putData(robotContainer.shooterPositionSub);
        SmartDashboard.putData(robotContainer.floorIntakeSub);
        robotContainer.visionSub.periodic(); 

        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        robotContainer.driveSub.stop();
        robotContainer.floorIntakeSub.stop();
        robotContainer.shooterSub.stopMotors();
        //robotContainer.shooterPositionSub.stopMotors(); // CHANGE - create a function to safely stop everything in this sub when we disbale
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
        // log auto data 
    }

    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void simulationInit() {
        robotContainer.driveSub.simulationInit();
    }

    @Override
    public void simulationPeriodic() {
        REVPhysicsSim.getInstance().run();

        var simPose = robotContainer.driveSub.getPoseWithoutVision();

        robotContainer.visionSub.simulationPeriodic(simPose);

        /*
        frc::Field2d& debugField = vision.GetSimDebugField();
        debugField.GetObject("EstimatedRobot")->SetPose(drivetrain.GetPose());
        debugField.GetObject("EstimatedRobotModules")
            ->SetPoses(drivetrain.GetModulePoses());
            */
    }
}
