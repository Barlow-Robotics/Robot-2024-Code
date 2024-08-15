package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Vision;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ShooterMountConstants;
import frc.robot.RobotContainer;

public class DynamicChoreo extends Command {
    private final Drive driveSub;

    private final Vision visionSub;
    private String mainPath;
    private String newAutoCaseNote;
    private String newAutoCaseNoNote;

    private Command currentCommand;
    private Command noteCommand;
    private Command noNoteCommand;

    private boolean path1Completed;


    public DynamicChoreo(String name, Vision visionSubsystem, Drive driveSub) {
        this.visionSub = visionSubsystem;
        this.driveSub = driveSub;
        initializeAutos(name);
        addRequirements(visionSubsystem);
    }

    @Override
    public void initialize() {
        currentCommand = driveSub.ChoreoAuto(mainPath);
        noteCommand = driveSub.ChoreoAutoWithoutReset(newAutoCaseNote);
        noNoteCommand = driveSub.ChoreoAutoWithoutReset(newAutoCaseNoNote);
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
            if (noteIsVisible() == 1) {
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
    public static double noteIsVisible() {
        return AutoConstants.noteIsVisible.get();
    }
}
