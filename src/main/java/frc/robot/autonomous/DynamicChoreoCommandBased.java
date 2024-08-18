package frc.robot.autonomous;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Vision;
import frc.robot.Constants.AutoConstants;

public class DynamicChoreoCommandBased extends SequentialCommandGroup {
    private String mainPath;
    private String newAutoCaseNote;
    private String newAutoCaseNoNote;

    public DynamicChoreoCommandBased(String name, Vision visionSubsystem, Drive driveSub) {
        initializeAutos(name);

        addCommands(
            driveSub.ChoreoAuto(mainPath),

            new ConditionalCommand(
                driveSub.ChoreoAutoWithoutReset(newAutoCaseNote), // If boolean = true
                driveSub.ChoreoAutoWithoutReset(newAutoCaseNoNote), // If boolean = false
                this::isNoteVisible // boolean
            )
        );

        addRequirements(visionSubsystem, driveSub);
    }

    private void initializeAutos(String name) {
        switch (name) {
            case "Routine D":
                this.mainPath = "[Dynamic] MidSpeaker MAIN";
                this.newAutoCaseNote = "[Dynamic] MidSpeaker Note";
                this.newAutoCaseNoNote = "[Dynamic] MidSpeaker NoNote";
                break;
            // case "Routine D":
            //     // Add initialization for other routines here
            //     break;
            default:
                throw new IllegalArgumentException("Unknown routine: " + name);
        }
    }

    private boolean isNoteVisible() {
        return AutoConstants.noteIsVisible.get() == 1;
    }
}