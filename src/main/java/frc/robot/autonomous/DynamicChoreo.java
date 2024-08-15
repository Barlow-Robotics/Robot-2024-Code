package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Vision;
import frc.robot.RobotContainer;

public class DynamicChoreo extends Command {

    private final Vision visionSub;
    private String mainPath;
    private String newAutoCaseNote;
    private String newAutoCaseNoNote;

    private Command currentCommand;
    private Command noteCommand;
    private Command noNoteCommand;

    private boolean path1Completed;


    public DynamicChoreo(String name, Vision visionSubsystem) {
        this.visionSub = visionSubsystem;
        initializeAutos(name);
        addRequirements(visionSubsystem);
    }

    @Override
    public void initialize() {
        currentCommand = RobotContainer.ChoreoAuto(mainPath);
        noteCommand = RobotContainer.ChoreoAuto(newAutoCaseNote);
        noNoteCommand = RobotContainer.ChoreoAuto(newAutoCaseNoNote);
        currentCommand.schedule();
        path1Completed = false;
    }

    private void initializeAutos(String name) {
        switch (name) {
            case "Routine C":
                this.mainPath = "[Dynamic] MidSpeaker MAIN";
                this.newAutoCaseNote = "[Dynamic] MidSpeaker Note";
                this.newAutoCaseNoNote = "[Dynamic] MidSpeaker NoNote";
                break;
            case "Routine D":
                break;
        }
    }
    

    @Override
    public void execute() {
        if (currentCommand != null && currentCommand.isFinished() && !path1Completed) {
            path1Completed = true;
            decideNextPath();
        }
    }

    private void decideNextPath() {
        if (path1Completed) {
            if (noteIsVisible()) {
                currentCommand.cancel();
                currentCommand = noteCommand;
            } else {
                currentCommand.cancel();
                currentCommand = noNoteCommand;
            }
            currentCommand.schedule();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        if (currentCommand != null) {
            currentCommand.end(interrupted);
        }
    }
    public static boolean noteIsVisible() {
        return true;
    }
}
