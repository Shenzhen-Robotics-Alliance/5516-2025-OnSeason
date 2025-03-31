package frc.robot.autos;

import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.auto.NamedCommands;
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

public class FourCoralStandard implements Auto {
    public static final Time WAIT_FOR_CORAL_TIMEOUT = Seconds.of(0.1);
    public static final Time WAIT_FOR_SUPER_STRUCTURE_TIMEOUT = Seconds.of(0.5);
    public static final Time SCORING_TIME = Seconds.of(0.45);

    private final boolean isRightSide;

    public FourCoralStandard(boolean isRightSide) {
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
                .andThen(robot.superStructure.moveToPose(SuperStructure.SuperStructurePose.PREPARE_TO_RUN));
        Command scoreCoral = robot.scoreCoral(SCORING_TIME.in(Seconds));
        NamedCommands.registerCommand("ElevatorUp", robot.moveToL4());

        int firstGoal = isRightSide ? 4 : 9;
        int secondGoal = isRightSide ? 3 : 10;
        int thirdGoal = isRightSide ? 2 : 11;
        int fourthGoal = isRightSide ? 1 : 0;

        // Score preloaded
        commandGroup.addCommands(Commands.runOnce(
                robot.superStructure.moveToPose(SuperStructure.SuperStructurePose.PREPARE_TO_RUN)::schedule));
        commandGroup.addCommands(Commands.runOnce(Commands.waitSeconds(0.6).andThen(robot.moveToL4())::schedule));
        commandGroup.addCommands(ReefAlignment.followPathAndAlign(
                robot, Auto.getChoreoPath("place preload", isRightSide), firstGoal, robot.moveToL4()));
        commandGroup.addCommands(Commands.waitUntil(robot.superStructure.atReference)
                .withTimeout(WAIT_FOR_SUPER_STRUCTURE_TIMEOUT.in(Seconds)));
        commandGroup.addCommands(Commands.runOnce(scoreCoral::schedule));
        commandGroup.addCommands(Commands.waitSeconds(0.2));

        // Grab second
        commandGroup.addCommands(
                followChoreoPath("grab second", RobotState.NavigationMode.SENSOR_LESS_ODOMETRY, isRightSide)
                        .deadlineFor(robot.superStructure
                                .moveToPose(SuperStructure.SuperStructurePose.IDLE)
                                .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)
                                .finallyDo(intakeCoral::schedule)
                                .asProxy()));
        commandGroup.addCommands(waitForIntake(robot));

        // Score second
        commandGroup.addCommands(ReefAlignment.followPathAndAlign(
                robot, Auto.getChoreoPath("place second", isRightSide), secondGoal, robot.moveToL4()));
        commandGroup.addCommands(Commands.waitUntil(robot.superStructure.atReference)
                .withTimeout(WAIT_FOR_SUPER_STRUCTURE_TIMEOUT.in(Seconds)));
        commandGroup.addCommands(Commands.runOnce(scoreCoral::schedule));
        commandGroup.addCommands(Commands.waitSeconds(0.2));

        // Grab third
        commandGroup.addCommands(
                followChoreoPath("grab third", RobotState.NavigationMode.SENSOR_LESS_ODOMETRY, isRightSide)
                        .deadlineFor(robot.superStructure
                                .moveToPose(SuperStructure.SuperStructurePose.IDLE)
                                .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)
                                .finallyDo(intakeCoral::schedule)
                                .asProxy()));
        commandGroup.addCommands(waitForIntake(robot));

        // Score Third
        commandGroup.addCommands(ReefAlignment.followPathAndAlign(
                robot, Auto.getChoreoPath("place third", isRightSide), thirdGoal, robot.moveToL4()));
        commandGroup.addCommands(Commands.waitUntil(robot.superStructure.atReference)
                .withTimeout(WAIT_FOR_SUPER_STRUCTURE_TIMEOUT.in(Seconds)));
        commandGroup.addCommands(Commands.runOnce(scoreCoral::schedule));
        commandGroup.addCommands(Commands.waitSeconds(0.2));

        // Grab Fourth
        commandGroup.addCommands(
                followChoreoPath("grab fourth", RobotState.NavigationMode.SENSOR_LESS_ODOMETRY, isRightSide)
                        .deadlineFor(robot.superStructure
                                .moveToPose(SuperStructure.SuperStructurePose.IDLE)
                                .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)
                                .finallyDo(intakeCoral::schedule)
                                .asProxy()));
        commandGroup.addCommands(waitForIntake(robot));

        // Score Fourth
        commandGroup.addCommands(ReefAlignment.followPathAndAlign(
                robot, Auto.getChoreoPath("place fourth", isRightSide), fourthGoal, robot.moveToL4()));
        commandGroup.addCommands(Commands.waitUntil(robot.superStructure.atReference)
                .withTimeout(WAIT_FOR_SUPER_STRUCTURE_TIMEOUT.in(Seconds)));
        commandGroup.addCommands(Commands.runOnce(scoreCoral::schedule));
        commandGroup.addCommands(Commands.waitSeconds(0.2));

        // Move back
        commandGroup.addCommands(robot.drive
                .run(() -> robot.drive.runRobotCentricChassisSpeeds(new ChassisSpeeds(-0.1, 0, 0)))
                .deadlineFor(robot.superStructure.moveToPose(SuperStructure.SuperStructurePose.IDLE))
                .withTimeout(0.5)
                .asProxy());

        System.out.println("auto command requires: ");
        for (Subsystem subsystem : commandGroup.getRequirements()) System.out.println("    " + subsystem);
        return commandGroup;
    }

    @Override
    public Pose2d getStartingPoseAtBlueAlliance() {
        Pose2d poseAtLeft = new Pose2d(7.0, 5.7, Rotation2d.fromDegrees(-135));
        return isRightSide ? Auto.flipLeftRight(poseAtLeft) : poseAtLeft;
    }
}
