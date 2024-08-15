package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ShooterMountConstants;
import frc.robot.subsystems.Vision;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.commands.PathPlannerAuto;

public class DynamicPathPlanner extends Command {
    private final String name;
    private final Vision visionSub;
    private List<PathPlannerAuto> autos;
    private SequentialCommandGroup commandGroup;
    private boolean autosModified = false;
    private int i = 0;
    private String newAutoCaseNote;
    private String newAutoCaseNoNote;
    private String[] autosToRemove;



    public DynamicPathPlanner(String name, Vision visionSub) {
        this.name = name;
        this.visionSub = visionSub;
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
                    this.newAutoCaseNote = "Speaker Front 3 Note";
                    this.newAutoCaseNoNote = "Four Note Auto";
                    this.autosToRemove = new String[]{"Speaker Left 2 Note"};
                    break;
                case "Routine B":
                    autos.add(new PathPlannerAuto("[Dynamic] Center Division"));
                    this.newAutoCaseNote = "[Dyamic] Center Continue"; 
                    this.newAutoCaseNoNote = "[Dyamic] Center Option 2";
                    this.autosToRemove = new String[]{"[Dynamic] Center Division"};
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

    private void initializeCommandGroup() {
        try {
            List<Command> commands = new ArrayList<>();
            for (PathPlannerAuto auto : autos) {
                commands.add(new PathPlannerAuto(auto.getName()));  // Ensure new instances
            }
            commandGroup = new SequentialCommandGroup(commands.toArray(new Command[0]));
        } catch (Exception e) {
            e.printStackTrace();
            System.out.println("Error initializing commandGroup: " + e.getMessage());
        }
    }

    @Override
    public void initialize() {
        this.autos = initializeAutos(name);
        initializeCommandGroup();
        autosModified = false;
        // Ensure initial command group scheduling
        scheduleCommandGroup();
    }

    @Override
    public void execute() {
        i+=1;
        if (noteIsVisible() == 1 && !autosModified && i >= 300 && isAutoFinished()) { // after 5 seconds do this
            cancelAndScheduleCommandGroup();            
            modifyAutosBasedOnVision(autosToRemove, newAutoCaseNote);
            autosModified = true;
            scheduleCommandGroup();
        }
        else if (!(noteIsVisible() == 1) && !autosModified && i >= 300 && isAutoFinished()) {
            cancelAndScheduleCommandGroup();            
            modifyAutosBasedOnVision(autosToRemove, newAutoCaseNoNote);
            autosModified = true;
            scheduleCommandGroup();
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (commandGroup != null) {
            commandGroup.end(interrupted);
        }
    }

    @Override
    public boolean isFinished() {
        return commandGroup != null && commandGroup.isFinished();
    }

    private void modifyAutosBasedOnVision(String[] autosToRemove, String newAutoName) {
        List<PathPlannerAuto> modifiedAutos = new ArrayList<>(autos);

        PathPlannerAuto autoToMove = new PathPlannerAuto(newAutoName);
        for (String autoName : autosToRemove) {
            modifiedAutos.removeIf(auto -> auto.getName().equals(autoName));
        }
        modifiedAutos.add(autoToMove);
        autos = new ArrayList<>(modifiedAutos);

        initializeCommandGroup();
    }

    private void scheduleCommandGroup() {
        if (commandGroup != null) {
            CommandScheduler.getInstance().schedule(commandGroup);

        }
    }

    private void cancelAndScheduleCommandGroup() {
        if (commandGroup != null) {
            CommandScheduler.getInstance().cancel(commandGroup); // Cancel the current command group
        }
    }
    public boolean isAutoFinished() {            
        return !CommandScheduler.getInstance().isScheduled(commandGroup);   
    }
    public static double noteIsVisible() {
        return AutoConstants.noteIsVisible.get();
    }

}