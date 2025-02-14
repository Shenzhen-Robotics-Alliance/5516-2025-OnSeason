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

public class ThreeCoralLeftSide implements Auto {
    @Override
    public Command getAutoCommand(RobotContainer robot) throws IOException, ParseException {
        final SequentialCommandGroup commandGroup = new SequentialCommandGroup();
        Command prepareToRunUp = robot.superStructure.moveToPose(SuperStructure.SuperStructurePose.LOW_SWAP_1);
        Command moveToL4 = robot.superStructure.moveToPose(SuperStructure.SuperStructurePose.SCORE_L4);

        // Score preloaded
        commandGroup.addCommands(Commands.runOnce(prepareToRunUp::schedule));
        commandGroup.addCommands(ReefAlignment.followPathAndAlign(
                        robot, PathPlannerPath.fromChoreoTrajectory("auto2 - place first"), 8, moveToL4)
                .asProxy());
        commandGroup.addCommands(robot.coralHolder.scoreCoral().asProxy());

        // Grab second
        commandGroup.addCommands(AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("auto2 - grab second"))
                .deadlineFor(robot.superStructure.moveToPose(SuperStructure.SuperStructurePose.INTAKE))
                .asProxy());
        commandGroup.addCommands(Commands.runOnce(robot.coralHolder.intakeCoralSequence()::schedule));
        commandGroup.addCommands(Commands.waitUntil(robot.coralHolder.hasCoral)
                .deadlineFor(Commands.print("waiting for coral...").repeatedly()));

        // Score second
        commandGroup.addCommands(Commands.runOnce(prepareToRunUp::schedule));
        commandGroup.addCommands(ReefAlignment.followPathAndAlign(
                        robot, PathPlannerPath.fromChoreoTrajectory("auto2 - place second"), 9, moveToL4)
                .asProxy());
        commandGroup.addCommands(robot.coralHolder.scoreCoral().asProxy());

        // Grab third
        commandGroup.addCommands(AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("auto2 - grab third"))
                .deadlineFor(robot.superStructure.moveToPose(SuperStructure.SuperStructurePose.INTAKE))
                .asProxy());
        commandGroup.addCommands(Commands.runOnce(robot.coralHolder.intakeCoralSequence()::schedule));
        commandGroup.addCommands(Commands.waitUntil(robot.coralHolder.hasCoral)
                .deadlineFor(Commands.print("waiting for coral...").repeatedly()));

        // Score Third
        commandGroup.addCommands(Commands.runOnce(prepareToRunUp::schedule));
        commandGroup.addCommands(ReefAlignment.followPathAndAlign(
                        robot, PathPlannerPath.fromChoreoTrajectory("auto2 - place third"), 10, moveToL4)
                .asProxy());
        commandGroup.addCommands(robot.coralHolder.scoreCoral().asProxy());

        // Grab fourth
        commandGroup.addCommands(AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("auto2 - grab fourth"))
                .deadlineFor(robot.superStructure.moveToPose(SuperStructure.SuperStructurePose.INTAKE))
                .asProxy());
        commandGroup.addCommands(Commands.runOnce(robot.coralHolder.intakeCoralSequence()::schedule));
        commandGroup.addCommands(Commands.waitUntil(robot.coralHolder.hasCoral)
                .deadlineFor(Commands.print("waiting for coral...").repeatedly()));

        // Score fourth
        commandGroup.addCommands(Commands.runOnce(prepareToRunUp::schedule));
        commandGroup.addCommands(ReefAlignment.followPathAndAlign(
                        robot, PathPlannerPath.fromChoreoTrajectory("auto2 - place fourth"), 11, moveToL4)
                .asProxy());
        commandGroup.addCommands(robot.coralHolder.scoreCoral().asProxy());

        System.out.println("auto command requires: ");
        for (Subsystem subsystem : commandGroup.getRequirements()) System.out.println("    " + subsystem);
        return commandGroup;
    }

    @Override
    public Pose2d getStartingPoseAtBlueAlliance() {
        return new Pose2d(7.843, 6.16, Rotation2d.fromDegrees(180));
    }
}
