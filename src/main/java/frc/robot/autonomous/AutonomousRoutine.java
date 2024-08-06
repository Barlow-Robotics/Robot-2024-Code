package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Vision;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.commands.PathPlannerAuto;

public class AutonomousRoutine extends Command {
    private final String name;
    private final Vision visionSub;
    private final List<PathPlannerAuto> autos;
    private SequentialCommandGroup commandGroup;
    private boolean autosModified = false;

    public AutonomousRoutine(String name, Vision visionSub) {
        this.name = name;
        this.visionSub = visionSub;
        this.autos = initializeAutos(name);
    }

    public String getName() {
        return name;
    }

    private List<PathPlannerAuto> initializeAutos(String name) {
        List<PathPlannerAuto> autos = new ArrayList<>();
        try {
            switch (name) {
                case "Routine A":
                    autos.add(new PathPlannerAuto("Speaker Left 2 Note"));
                    autos.add(new PathPlannerAuto("Four Note Auto"));
                    break;
                case "Routine B":
                    // autos.add(new PathPlannerAuto("(pivot) Front-Speaker to Note"));
                    break;
                default:
                    autos.add(new PathPlannerAuto("BASIC"));
                    break;
            }
        } catch (Exception e) {
            e.printStackTrace();
            System.out.println("Error initializing autos: " + e.getMessage());
        }
        return autos;
    }

    @Override
    public void initialize() {
        if (!visionSub.noteIsVisible() && !autosModified) {
            modifyAutosBasedOnVision();
            autosModified = true;
        }

        try {
            List<Command> commands = new ArrayList<>(autos);
            commandGroup = new SequentialCommandGroup(commands.toArray(new Command[0]));
            commandGroup.schedule();
        } catch (Exception e) {
            e.printStackTrace();
            System.out.println("Error initializing commandGroup in initialize: " + e.getMessage());
        }
    }

    @Override
    public void execute() {
        // commandGroup.execute();
    }

    @Override
    public void end(boolean interrupted) {
        commandGroup.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return commandGroup.isFinished();
    }

    private void modifyAutosBasedOnVision() {
        try {
            // PathPlannerAuto autoToMove = new PathPlannerAuto("pathToMove");
            // autos.remove(autoToMove);
            // autos.add(0, autoToMove);
        } catch (Exception e) {
            e.printStackTrace();
            System.out.println("Error modifying autos based on vision: " + e.getMessage());
        }
    }
}
