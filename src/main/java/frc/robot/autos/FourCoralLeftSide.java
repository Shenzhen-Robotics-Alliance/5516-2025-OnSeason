package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;
import frc.robot.commands.reefscape.ReefAlignment;
import frc.robot.subsystems.superstructure.SuperStructure;
import java.io.IOException;
import org.json.simple.parser.ParseException;

public class FourCoralLeftSide implements Auto {
    @Override
    public Command getAutoCommand(RobotContainer robot) throws IOException, ParseException {
        final SequentialCommandGroup commandGroup = new SequentialCommandGroup();
        Command prepareToRunUp = // robot.superStructure.moveToPose(SuperStructure.SuperStructurePose.LOW_SWAP_1);
                Commands.none();
        Command moveToL4 = robot.superStructure.moveToPose(SuperStructure.SuperStructurePose.SCORE_L4);
        Command moveToL3 = robot.superStructure.moveToPose(SuperStructure.SuperStructurePose.SCORE_L3);

        // Score preloaded
        commandGroup.addCommands(Commands.runOnce(prepareToRunUp::schedule));
        commandGroup.addCommands(ReefAlignment.followPathAndAlign(
                        robot, PathPlannerPath.fromChoreoTrajectory("place first"), 8, moveToL4)
                .deadlineFor(robot.coralHolder.keepCoralShuffledForever())
                .asProxy());
        commandGroup.addCommands(robot.coralHolder.scoreCoral().asProxy());

        // Grab second
        commandGroup.addCommands(AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("grab second"))
                .deadlineFor(robot.superStructure.moveToPose(SuperStructure.SuperStructurePose.INTAKE))
                .asProxy());
        commandGroup.addCommands(Commands.runOnce(robot.coralHolder.intakeCoralSequence()::schedule));
        commandGroup.addCommands(Commands.waitUntil(robot.coralHolder.hasCoral)
                .deadlineFor(Commands.print("waiting for coral...").repeatedly()));

        // Score second
        commandGroup.addCommands(Commands.runOnce(prepareToRunUp::schedule));
        commandGroup.addCommands(ReefAlignment.followPathAndAlign(
                        robot, PathPlannerPath.fromChoreoTrajectory("place second"), 10, moveToL4)
                .deadlineFor(robot.coralHolder.keepCoralShuffledForever())
                .asProxy());
        commandGroup.addCommands(robot.coralHolder.scoreCoral().asProxy());

        // Grab third
        commandGroup.addCommands(AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("grab third"))
                .deadlineFor(robot.superStructure.moveToPose(SuperStructure.SuperStructurePose.INTAKE))
                .asProxy());
        commandGroup.addCommands(Commands.runOnce(robot.coralHolder.intakeCoralSequence()::schedule));
        commandGroup.addCommands(Commands.waitUntil(robot.coralHolder.hasCoral)
                .deadlineFor(Commands.print("waiting for coral...").repeatedly()));

        // Score Third
        commandGroup.addCommands(Commands.runOnce(prepareToRunUp::schedule));
        commandGroup.addCommands(ReefAlignment.followPathAndAlign(
                        robot, PathPlannerPath.fromChoreoTrajectory("place third"), 11, moveToL4)
                .deadlineFor(robot.coralHolder.keepCoralShuffledForever())
                .asProxy());
        commandGroup.addCommands(robot.coralHolder.scoreCoral().asProxy());

        // Grab fourth
        commandGroup.addCommands(AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("grab fourth"))
                .deadlineFor(robot.superStructure.moveToPose(SuperStructure.SuperStructurePose.INTAKE))
                .asProxy());
        commandGroup.addCommands(Commands.runOnce(robot.coralHolder.intakeCoralSequence()::schedule));
        commandGroup.addCommands(Commands.waitUntil(robot.coralHolder.hasCoral)
                .deadlineFor(Commands.print("waiting for coral...").repeatedly()));

        // Score fourth
        commandGroup.addCommands(Commands.runOnce(prepareToRunUp::schedule));
        commandGroup.addCommands(ReefAlignment.followPathAndAlign(
                        robot, PathPlannerPath.fromChoreoTrajectory("place fourth"), 11, moveToL3)
                .deadlineFor(robot.coralHolder.keepCoralShuffledForever())
                .asProxy());
        commandGroup.addCommands(robot.coralHolder.scoreCoral().asProxy());

        System.out.println("auto command requires: ");
        for (Subsystem subsystem : commandGroup.getRequirements()) System.out.println("    " + subsystem);
        return commandGroup;
    }

    @Override
    public Pose2d getStartingPoseAtBlueAlliance() {
        return new Pose2d(7.34, 5.66, Rotation2d.fromDegrees(180));
    }
}
