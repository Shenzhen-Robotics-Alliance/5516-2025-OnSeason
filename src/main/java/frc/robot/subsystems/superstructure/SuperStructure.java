package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.subsystems.superstructure.arm.Arm;
import frc.robot.subsystems.superstructure.arm.ArmConstants;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.ElevatorConstants;
import java.util.*;

public class SuperStructure {
    private static final TrapezoidProfile elevatorProfile = new TrapezoidProfile(ElevatorConstants.PROFILE_CONSTRAINS);
    private static final TrapezoidProfile armProfile = new TrapezoidProfile(ArmConstants.PROFILE_CONSTRAINS);

    /**
     * Represents a pose of the super structure
     *
     * <p>The pose of the super structure is the combination of the elevator height and arm position
     */
    public enum SuperStructurePose {
        // Useful poses
        IDLE(Meters.of(0), Degrees.of(116)),
        INTAKE(Centimeters.of(5), Degrees.of(132)),
        SCORE_L2(Meters.of(0.2), Degrees.of(116)),
        SCORE_L3(Meters.of(0.62), Degrees.of(116)),
        SCORE_L4(Meters.of(1.32), Degrees.of(85)),

        // Swap poses that serve as interior waypoints
        // (don't run them)
        // Allow Arm to swing down at zero height
        PREPARE_TO_RUN_UP(Meters.zero(), Degrees.of(55)),

        // At 0.3 meters height, allow arm to swing up and down
        LOW_SWAP_1(Meters.of(0.3), Degrees.of(116)),
        LOW_SWAP_2(Meters.of(0.3), Degrees.of(55)),

        // At 1.32 Meters there is a swap pose to run to L4
        HIGH_SWAP(Meters.of(1.32), Degrees.of(55));

        public final Distance elevatorHeight;
        public final Angle armAngle;

        SuperStructurePose(Distance elevatorHeight, Angle armAngle) {
            this.elevatorHeight = elevatorHeight;
            this.armAngle = armAngle;
        }
    }

    /**
     * Represents a link between two super structure poses
     *
     * <p>If two poses are linked, it means that the super structure can move from pose1 to pose2 directly (without
     * hitting dead-bands).
     */
    public record PoseLink(SuperStructurePose pose1, SuperStructurePose pose2) {
        /** Calculates the amount of time needed for the super structure to move */
        public double timeSeconds() {
            TrapezoidProfile.State elevatorState = new TrapezoidProfile.State(pose1.elevatorHeight.in(Meters), 0);
            TrapezoidProfile.State armState = new TrapezoidProfile.State(pose1.armAngle.in(Radians), 0);
            elevatorProfile.calculate(Robot.defaultPeriodSecs, elevatorState, elevatorState);
            armProfile.calculate(Robot.defaultPeriodSecs, armState, armState);
            double elevatorTime = elevatorProfile.timeLeftUntil(pose2.elevatorHeight.in(Meters));
            double armTime = armProfile.timeLeftUntil(pose2.armAngle.in(Radians));
            return Math.max(elevatorTime, armTime);
        }

        public Optional<SuperStructurePose> otherEdge(SuperStructurePose oneEdge) {
            if (oneEdge == pose1) return Optional.of(pose2);
            if (oneEdge == pose2) return Optional.of(pose1);
            else return Optional.empty();
        }
    }

    public static List<PoseLink> LINKS = List.of(
            // We can run to intake / l2 / l3 directly from idle
            new PoseLink(SuperStructurePose.IDLE, SuperStructurePose.INTAKE),
            new PoseLink(SuperStructurePose.IDLE, SuperStructurePose.SCORE_L2),
            new PoseLink(SuperStructurePose.IDLE, SuperStructurePose.SCORE_L3),

            // From Idle we can go to a few swap poses
            new PoseLink(SuperStructurePose.IDLE, SuperStructurePose.PREPARE_TO_RUN_UP),
            new PoseLink(SuperStructurePose.IDLE, SuperStructurePose.LOW_SWAP_1),
            new PoseLink(SuperStructurePose.IDLE, SuperStructurePose.LOW_SWAP_2),

            // From L2 we can run to a few swap poses
            new PoseLink(SuperStructurePose.SCORE_L2, SuperStructurePose.PREPARE_TO_RUN_UP),
            new PoseLink(SuperStructurePose.SCORE_L2, SuperStructurePose.LOW_SWAP_1),
            new PoseLink(SuperStructurePose.SCORE_L2, SuperStructurePose.LOW_SWAP_2),

            // From some lower swap poses we can run to the higher swap poses
            new PoseLink(SuperStructurePose.LOW_SWAP_2, SuperStructurePose.HIGH_SWAP),
            new PoseLink(SuperStructurePose.PREPARE_TO_RUN_UP, SuperStructurePose.HIGH_SWAP),

            // To run to L4 we must go throw high-swap
            new PoseLink(SuperStructurePose.HIGH_SWAP, SuperStructurePose.SCORE_L4));

    private final Elevator elevator;
    private final Arm arm;

    private SuperStructurePose currentPose;

    public final Trigger atReference;

    public SuperStructure(Elevator elevator, Arm arm) {
        this.elevator = elevator;
        this.arm = arm;
        this.currentPose = SuperStructurePose.IDLE;

        atReference = new Trigger(
                () -> elevator.atReference(currentPose.elevatorHeight) && arm.atReference(currentPose.armAngle));

        for (PoseLink link : LINKS) System.out.println("time: " + link.timeSeconds());
    }

    private Command runPose(SuperStructurePose pose) {
        return elevator.moveToPosition(pose.elevatorHeight).alongWith(arm.moveToPosition(pose.armAngle));
    }

    public Command moveToPose(SuperStructurePose pose) {
        return Commands.defer(() -> generateMoveToPoseCommand(pose), Set.of(elevator, arm));
    }

    private Command generateMoveToPoseCommand(SuperStructurePose pose) {
        Optional<List<SuperStructurePose>> trajectory = getTrajectory(pose);
        if (trajectory.isEmpty()) return Commands.none();
        return Commands.sequence(trajectory.get().stream().map(this::runPose).toArray(Command[]::new));
    }

    private static final int loopNumLimit = 20;
    /**
     * Finds the minimum-time trajectory to move from a pose to a target pose.
     *
     * <p>This uses a shortest path algorithm (Dijkstra) to find the optimal path considering the time to move between
     * poses.
     */
    public static Optional<List<SuperStructurePose>> getTrajectory(
            SuperStructurePose startingPose, SuperStructurePose targetPose) {
        if (startingPose.equals(targetPose)) return Optional.of(new ArrayList<>());

        Set<SuperStructurePose> unvisited = new HashSet<>(Set.of(SuperStructurePose.values()));
        // The path that leads to a node with minimum time
        Map<SuperStructurePose, PoseLink> minimumTimePathToNode = new HashMap<>();
        Map<SuperStructurePose, Double> minimumTimeToPose = new HashMap<>();
        for (SuperStructurePose pose : SuperStructurePose.values())
            minimumTimeToPose.put(pose, Double.POSITIVE_INFINITY);
        minimumTimeToPose.put(startingPose, 0.0);

        SuperStructurePose currentNode = startingPose;
        // limit loop time to avoid code crashes
        for (int i = 0; i < loopNumLimit; i++) {
            // If we've already reached target
            if (currentNode.equals(targetPose)) {
                List<SuperStructurePose> trajectory = new ArrayList<>();
                SuperStructurePose tmp = currentNode;
                for (int j = 0; j < loopNumLimit; j++) {
                    System.out.println("tmp: " + tmp);
                    trajectory.add(0, tmp);
                    if (tmp == startingPose) return Optional.of(trajectory);
                    if (!minimumTimePathToNode.containsKey(tmp)) {
                        DriverStation.reportError("Internal Error while tracing back trajectory", true);
                        return Optional.empty();
                    }
                    Optional<SuperStructurePose> otherEdge =
                            minimumTimePathToNode.get(tmp).otherEdge(tmp);
                    if (otherEdge.isEmpty()) {
                        DriverStation.reportError("Internal Error while tracing back trajectory", true);
                        return Optional.empty();
                    }
                    tmp = otherEdge.get();
                }
                DriverStation.reportError(
                        "Internal Error: destination reached, but cannot traceback trajectory in " + loopNumLimit
                                + " iterations",
                        true);
                return Optional.empty();
            }

            Optional<PoseLink> minimumTimeLink = Optional.empty();
            for (PoseLink link : LINKS) {
                Optional<SuperStructurePose> otherNode = link.otherEdge(currentNode);
                if (otherNode.isEmpty()) continue;
                if (!unvisited.contains(otherNode.get())) continue;

                double newTime = minimumTimeToPose.get(currentNode) + link.timeSeconds();
                if (newTime < minimumTimeToPose.get(otherNode.get())) {
                    minimumTimePathToNode.put(otherNode.get(), link);
                    minimumTimeToPose.put(otherNode.get(), newTime);
                }

                if (minimumTimeLink.isEmpty() || minimumTimeLink.get().timeSeconds() > link.timeSeconds())
                    minimumTimeLink = Optional.of(link);
            }

            unvisited.remove(currentNode);
            if (minimumTimeLink.isPresent()
                    && minimumTimeLink.get().otherEdge(currentNode).isPresent())
                currentNode = minimumTimeLink.get().otherEdge(currentNode).get();
            else break;
        }

        DriverStation.reportError(
                "Failed to plan super structure trajectory in " + loopNumLimit + " iterations!", true);
        return Optional.empty();
    }

    public Optional<List<SuperStructurePose>> getTrajectory(SuperStructurePose targetPose) {
        return getTrajectory(currentPose, targetPose);
    }
}
