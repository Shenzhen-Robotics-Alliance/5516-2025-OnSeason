package frc.robot.autos;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.commands.reefscape.ReefAlignment;
import frc.robot.constants.RobotMode;
import frc.robot.subsystems.superstructure.SuperStructure;
import java.io.IOException;
import org.json.simple.parser.ParseException;

public class ThreeCoralShort implements Auto {
    public static final Time WAIT_FOR_CORAL_TIMEOUT = Seconds.of(0.8);
    public static final Time WAIT_FOR_SUPER_STRUCTURE_TIMEOUT = Seconds.of(0.5);
    public static final Time SCORING_TIME = Seconds.of(0.5);

    private final boolean isRightSide;

    public ThreeCoralShort(boolean isRightSide) {
        this.isRightSide = isRightSide;
    }

    private static Command runRobotBackwardsSlow(RobotContainer robot) {
        return robot.drive
                .run(() -> robot.drive.runRobotCentricChassisSpeeds(new ChassisSpeeds(-0.5, 0, 0)))
                .asProxy();
    }

    private static Command waitForIntake(RobotContainer robot) {
        return Commands.waitUntil(robot.coralHolder.hasCoral.and(() -> Robot.CURRENT_ROBOT_MODE != RobotMode.SIM))
                .withTimeout(WAIT_FOR_CORAL_TIMEOUT)
                .deadlineFor(runRobotBackwardsSlow(robot))
                .deadlineFor(Commands.print("waiting for coral...").repeatedly());
    }

    @Override
    public Command getAutoCommand(RobotContainer robot) throws IOException, ParseException {
        final SequentialCommandGroup commandGroup = new SequentialCommandGroup();
        Command intakeCoral = robot.coralHolder
                .intakeCoralSequence()
                .andThen(robot.superStructure.moveToPose(SuperStructure.SuperStructurePose.IDLE));

        int firstGoal = isRightSide ? 4 : 9;
        int secondGoal = isRightSide ? 3 : 10;
        int thirdGoalAndFourthGoal = isRightSide ? 2 : 11;

        // Score preloaded
        Command raiseElevatorForPreloaded = robot.superStructure
                .moveToPose(SuperStructure.SuperStructurePose.SCORE_L4)
                .deadlineFor(Commands.waitSeconds(0.3).andThen(robot.coralHolder.keepCoralShuffledForever()))
                // only raise elevator if coral in place to avoid getting jammed
                .onlyIf(robot.coralHolder.secondSensor)
                .asProxy();
        Command waitAndRaiseElevator = Commands.waitSeconds(0.7).andThen(raiseElevatorForPreloaded);
        commandGroup.addCommands(Commands.runOnce(waitAndRaiseElevator::schedule));
        commandGroup.addCommands(ReefAlignment.followPathAndAlign(
                robot, Auto.getChoreoPath("place first", isRightSide), firstGoal, Commands.none()));
        commandGroup.addCommands(Commands.waitUntil(robot.superStructure.atReference)
                .withTimeout(WAIT_FOR_SUPER_STRUCTURE_TIMEOUT.in(Seconds)));
        commandGroup.addCommands(robot.scoreCoral(SCORING_TIME.in(Seconds)));

        // Grab second
        commandGroup.addCommands(
                followChoreoPath("grab second", RobotState.NavigationMode.SENSOR_LESS_ODOMETRY, isRightSide)
                        .deadlineFor(robot.superStructure
                                .moveToPose(SuperStructure.SuperStructurePose.INTAKE)
                                .finallyDo(intakeCoral::schedule)
                                .asProxy()));
        commandGroup.addCommands(waitForIntake(robot));

        // Score second
        commandGroup.addCommands(ReefAlignment.followPathAndAlign(
                robot, Auto.getChoreoPath("place second", isRightSide), secondGoal, Commands.none()));
        commandGroup.addCommands(Commands.waitUntil(robot.superStructure.atReference)
                .withTimeout(WAIT_FOR_SUPER_STRUCTURE_TIMEOUT.in(Seconds)));
        commandGroup.addCommands(robot.scoreCoral(SCORING_TIME.in(Seconds)));

        // Grab third
        commandGroup.addCommands(
                followChoreoPath("grab third", RobotState.NavigationMode.SENSOR_LESS_ODOMETRY, isRightSide)
                        .deadlineFor(robot.superStructure
                                .moveToPose(SuperStructure.SuperStructurePose.INTAKE)
                                .finallyDo(intakeCoral::schedule)
                                .asProxy()));
        commandGroup.addCommands(waitForIntake(robot));

        // Score Third
        commandGroup.addCommands(ReefAlignment.followPathAndAlign(
                robot, Auto.getChoreoPath("place third", isRightSide), thirdGoalAndFourthGoal, Commands.none()));
        commandGroup.addCommands(Commands.waitUntil(robot.superStructure.atReference)
                .withTimeout(WAIT_FOR_SUPER_STRUCTURE_TIMEOUT.in(Seconds)));
        commandGroup.addCommands(robot.scoreCoral(SCORING_TIME.in(Seconds)));

        // Grab fourth
        commandGroup.addCommands(
                followChoreoPath("grab fourth", RobotState.NavigationMode.SENSOR_LESS_ODOMETRY, isRightSide)
                        .deadlineFor(robot.superStructure
                                .moveToPose(SuperStructure.SuperStructurePose.IDLE)
                                .asProxy()));
        //        commandGroup.addCommands(AutoBuilder.followPath(Auto.getChoreoPath("grab fourth", isRightSide))
        //                .deadlineFor(robot.superStructure
        //                        .moveToPose(SuperStructure.SuperStructurePose.INTAKE)
        //                        .asProxy()
        //                        .finallyDo(intakeCoral::schedule))
        //                .asProxy());
        //        commandGroup.addCommands(Commands.waitUntil(robot.coralHolder.hasCoral)
        //                .deadlineFor(runRobotBackwardsSlow(robot))
        //                .deadlineFor(Commands.print("waiting for coral...").repeatedly()));
        //
        //        // Score fourth
        //        commandGroup.addCommands(ReefAlignment.followPathAndAlign(
        //                        robot, Auto.getChoreoPath("place fourth", isRightSide), thirdGoalAndFourthGoal,
        // moveToL3)
        //                .asProxy());
        //        commandGroup.addCommands(
        //                Commands.waitUntil(robot.superStructure.atReference).withTimeout(superStructTimeOutSeconds));
        //        commandGroup.addCommands(robot.coralHolder.scoreCoral().asProxy());

        System.out.println("auto command requires: ");
        for (Subsystem subsystem : commandGroup.getRequirements()) System.out.println("    " + subsystem);
        return commandGroup;
    }

    @Override
    public Pose2d getStartingPoseAtBlueAlliance() {
        Pose2d poseAtLeft = new Pose2d(7.1, 6.15, Rotation2d.fromDegrees(-135));
        return isRightSide ? Auto.flipLeftRight(poseAtLeft) : poseAtLeft;
    }
}
