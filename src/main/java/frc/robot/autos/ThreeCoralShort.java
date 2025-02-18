package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;
import frc.robot.commands.reefscape.ReefAlignment;
import frc.robot.subsystems.superstructure.SuperStructure;
import java.io.IOException;
import org.json.simple.parser.ParseException;

public class ThreeCoralShort implements Auto {
    private final boolean isRightSide;

    public ThreeCoralShort(boolean isRightSide) {
        this.isRightSide = isRightSide;
    }

    private static Command runRobotBackwardsSlow(RobotContainer robot) {
        return robot.drive
                .run(() -> robot.drive.runRobotCentricChassisSpeeds(new ChassisSpeeds(-0.3, 0, 0)))
                .asProxy();
    }

    @Override
    public Command getAutoCommand(RobotContainer robot) throws IOException, ParseException {
        final SequentialCommandGroup commandGroup = new SequentialCommandGroup();
        Command moveToL3 = robot.superStructure.moveToPose(SuperStructure.SuperStructurePose.SCORE_L3);
        Command intakeCoral = robot.coralHolder
                .intakeCoralSequence()
                .andThen(robot.superStructure.moveToPose(SuperStructure.SuperStructurePose.IDLE));

        int firstGoal = isRightSide ? 5 : 8;
        int secondGoal = isRightSide ? 3 : 10;
        int thirdGoalAndFourthGoal = isRightSide ? 2 : 11;
        double superStructTimeOutSeconds = 10;

        // Score preloaded
        commandGroup.addCommands(ReefAlignment.followPathAndAlign(
                        robot, Auto.getChoreoPath("place first", isRightSide), firstGoal, Commands.none())
                .asProxy());
        commandGroup.addCommands(
                Commands.waitUntil(robot.superStructure.atReference).withTimeout(superStructTimeOutSeconds));
        commandGroup.addCommands(robot.coralHolder.scoreCoral().asProxy());

        // Grab second
        commandGroup.addCommands(AutoBuilder.followPath(Auto.getChoreoPath("grab second", isRightSide))
                .deadlineFor(robot.superStructure.moveToPose(SuperStructure.SuperStructurePose.INTAKE))
                .asProxy());
        commandGroup.addCommands(Commands.runOnce(intakeCoral::schedule));
        commandGroup.addCommands(Commands.waitUntil(robot.coralHolder.hasCoral)
                .deadlineFor(runRobotBackwardsSlow(robot))
                .deadlineFor(Commands.print("waiting for coral...").repeatedly()));

        // Score second
        commandGroup.addCommands(ReefAlignment.followPathAndAlign(
                        robot, Auto.getChoreoPath("place second", isRightSide), secondGoal, Commands.none())
                .asProxy());
        commandGroup.addCommands(
                Commands.waitUntil(robot.superStructure.atReference).withTimeout(superStructTimeOutSeconds));
        commandGroup.addCommands(robot.coralHolder.scoreCoral().asProxy());

        // Grab third
        commandGroup.addCommands(AutoBuilder.followPath(Auto.getChoreoPath("grab third", isRightSide))
                .deadlineFor(robot.superStructure.moveToPose(SuperStructure.SuperStructurePose.INTAKE))
                .asProxy());
        commandGroup.addCommands(Commands.runOnce(intakeCoral::schedule));
        commandGroup.addCommands(Commands.waitUntil(robot.coralHolder.hasCoral)
                .deadlineFor(runRobotBackwardsSlow(robot))
                .deadlineFor(Commands.print("waiting for coral...").repeatedly()));

        // Score Third
        commandGroup.addCommands(ReefAlignment.followPathAndAlign(
                        robot, Auto.getChoreoPath("place third", isRightSide), thirdGoalAndFourthGoal, Commands.none())
                .asProxy());
        commandGroup.addCommands(
                Commands.waitUntil(robot.superStructure.atReference).withTimeout(superStructTimeOutSeconds));
        commandGroup.addCommands(robot.coralHolder.scoreCoral().asProxy());

        // Grab fourth
        commandGroup.addCommands(AutoBuilder.followPath(Auto.getChoreoPath("grab fourth", isRightSide))
                .deadlineFor(robot.superStructure.moveToPose(SuperStructure.SuperStructurePose.INTAKE))
                .asProxy());
        commandGroup.addCommands(Commands.runOnce(intakeCoral::schedule));
        commandGroup.addCommands(Commands.waitUntil(robot.coralHolder.hasCoral)
                .deadlineFor(runRobotBackwardsSlow(robot))
                .deadlineFor(Commands.print("waiting for coral...").repeatedly()));

        // Score fourth
        commandGroup.addCommands(ReefAlignment.followPathAndAlign(
                        robot, Auto.getChoreoPath("place fourth", isRightSide), thirdGoalAndFourthGoal, moveToL3)
                .asProxy());
        commandGroup.addCommands(
                Commands.waitUntil(robot.superStructure.atReference).withTimeout(superStructTimeOutSeconds));
        commandGroup.addCommands(robot.coralHolder.scoreCoral().asProxy());

        System.out.println("auto command requires: ");
        for (Subsystem subsystem : commandGroup.getRequirements()) System.out.println("    " + subsystem);
        return commandGroup;
    }

    @Override
    public Pose2d getStartingPoseAtBlueAlliance() {
        Pose2d poseAtLeft = new Pose2d(7.2, 5.8, Rotation2d.fromDegrees(-135));
        return isRightSide ? Auto.flipLeftRight(poseAtLeft) : poseAtLeft;
    }
}
