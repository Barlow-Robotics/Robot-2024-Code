package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Vision;
import frc.robot.Constants.AutoConstants;

public class DynamicChoreoCommandBased2 extends Command {
    private final Drive driveSubsystem;
    private final Vision visionSubsystem;
    private String mainPath;
    private String newAutoCaseNote;
    private String newAutoCaseNoNote;
    private boolean pathSwitched = false;
    // private Pose2d lastPose2d;
    // private Pose2d currPose2d;

    public DynamicChoreoCommandBased2(String name, Vision visionSubsystem, Drive driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        initializeAutos(name);
        addRequirements(driveSubsystem, visionSubsystem);
    }

    @Override
    public void initialize() {
        // Start both the main path and the note path together
        driveSubsystem.ChoreoAuto(mainPath).andThen(
            driveSubsystem.ChoreoAutoWithoutReset(newAutoCaseNote)
        ).schedule();
        // lastPose2d = driveSubsystem.getPose();
    }

    @Override
    public void execute() {
        // if (currPose2d != null) {
        //     lastPose2d = currPose2d;
        // }
        // currPose2d = driveSubsystem.getPose();
        // Continuously monitor if the vision detects a note
        System.out.println(driveSubsystem.isMoving());
        if (!pathSwitched && !isNoteVisible()) {
            // If the note is not visible, switch to the NoNote path
            driveSubsystem.ChoreoAutoWithoutReset(newAutoCaseNoNote).schedule();
            pathSwitched = true; // Ensure the switch happens only once
        }
    }

    @Override
    public boolean isFinished() {
        // The command finishes when the entire sequence is complete
        return pathSwitched && !driveSubsystem.isMoving();
    }

    private void initializeAutos(String name) {
        switch (name) {
            case "Routine E":
                this.mainPath = "[Dynamic] MidSpeaker MAIN";
                this.newAutoCaseNote = "[Dynamic] MidSpeaker Note";
                this.newAutoCaseNoNote = "[Dynamic] MidSpeaker NoNote";
                break;
            default:
                throw new IllegalArgumentException("Unknown routine: " + name);
        }
    }

    private boolean isNoteVisible() {
        return AutoConstants.noteIsVisible.get() == 1;
    }
}