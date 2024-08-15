package frc.robot.autonomous;

import java.util.List;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Vision;
import frc.robot.RobotContainer;
import frc.robot.autonomous.DynamicPathPlanner;

public class DynamicChoreo extends Command {

    private final Vision visionSub;
    private final String name;
    private String mainPath;
    private String newAutoCaseNote;
    private String newAutoCaseNoNote;

    private Command currentCommand;
    private Command noteCommand;
    private Command noNoteCommand;

    private boolean path1Completed = false;

    public DynamicChoreo(String name, Vision visionSubsystem) {
        this.visionSub = visionSubsystem;
        this.name = name;
        initializeAutos(name);

        addRequirements(visionSubsystem);
    }

    @Override
    public void initialize() {
        currentCommand = RobotContainer.ChoreoAuto(mainPath);
        noteCommand = RobotContainer.ChoreoAuto(newAutoCaseNote);
        noNoteCommand = RobotContainer.ChoreoAuto(newAutoCaseNoNote);
        currentCommand.schedule();
    }

    private void initializeAutos(String name) {
        switch (name) {
            case "Routine C":
                this.mainPath = "[Dynamic] MidSpeaker MAIN";
                this.newAutoCaseNote = "[Dynamic] MidSpeaker Note";
                this.newAutoCaseNoNote = "[Dynamic] MidSpeaker NoNote";
                break;
            case "Routine D":
                // this.mainPath = "Speaker Front 3 Note";
                // this.newAutoCaseNote = "Four Note Auto";
                // this.newAutoCaseNoNote = "Speaker Left 2 Note";
                break;
        }
    }
    

    @Override
    public void execute() {
        System.out.println(currentCommand.isFinished());
        if (currentCommand != null && currentCommand.isFinished()) {
            // Path1 is done
            System.out.println("Path1 is done");
            path1Completed = true;
            decideNextPath();
        }
    }

    private void decideNextPath() {
        if (path1Completed) {
            if (DynamicPathPlanner.noteIsVisible() ) {
                // Switch to Path3
                System.out.println(newAutoCaseNote);
                currentCommand = noteCommand;
            } else {
                // Continue with Path2
                currentCommand.cancel(); // Cancel the current command
                currentCommand = noNoteCommand;
            }
            System.out.println(currentCommand);
            currentCommand.schedule();
        }
    }
    public boolean isAutoFinished() {            
        return !CommandScheduler.getInstance().isScheduled(currentCommand);   
    }

    @Override
    public boolean isFinished() {
        // Finish when the final trajectory is completed
        return currentCommand != null && currentCommand.isFinished()        ;
        // return path1Completed && !currentCommand.isSche/duled();
    }

    @Override
    public void end(boolean interrupted) {
        if (currentCommand != null) {
            currentCommand.end(interrupted);
        }
    }
}
