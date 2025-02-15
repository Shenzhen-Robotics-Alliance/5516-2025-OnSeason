package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.superstructure.arm.Arm;
import frc.robot.subsystems.superstructure.arm.ArmConstants;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.ElevatorConstants;
import java.util.*;

public class SuperStructure {
    /**
     * Represents a pose of the super structure
     *
     * <p>The pose of the super structure is the combination of the elevator height and arm position
     */
    public enum SuperStructurePose {
        // Useful poses
        IDLE(Meters.of(0), Degrees.of(110)),
        INTAKE(Centimeters.of(3.5), Degrees.of(136)),
        SCORE_L2(Meters.of(0.2), Degrees.of(110)),
        SCORE_L3(Meters.of(0.64), Degrees.of(110)),
        SCORE_L4(Meters.of(1.28), Degrees.of(98)),

        // Swap poses that serve as interior waypoints
        // (don't run them)
        // Allow Arm to swing down at zero height

        // At 0.3 meters height, allow arm to swing up and down
        LOW_SWAP_1(Meters.of(0.3), Degrees.of(110)),
        LOW_SWAP_2(Meters.of(0.3), Degrees.of(55)),

        // Swap pose to run to L4
        HIGH_SWAP(Meters.of(1.28), Degrees.of(110)),

        // Legacy L4 Scoring Poses (for dev bot)
        SCORE_L4_LEGACY(Meters.of(1.32), Degrees.of(85)),
        HIGH_SWAP_LEGACY(Meters.of(1.32), Degrees.of(55)),
        PREPARE_TO_RUN_UP_LEGACY(Meters.zero(), Degrees.of(55));

        public final Distance elevatorHeight;
        public final Angle armAngle;

        SuperStructurePose(Distance elevatorHeight, Angle armAngle) {
            this.elevatorHeight = elevatorHeight;
            this.armAngle = armAngle;
        }
    }

    public static List<PoseLink> LINKS = List.of(
            // We can run to intake / l2 / l3 directly from idle
            new PoseLink(SuperStructurePose.IDLE, SuperStructurePose.INTAKE),
            new PoseLink(SuperStructurePose.IDLE, SuperStructurePose.SCORE_L2),
            new PoseLink(SuperStructurePose.IDLE, SuperStructurePose.SCORE_L3),
            new PoseLink(SuperStructurePose.SCORE_L2, SuperStructurePose.SCORE_L3),

            // From a few swap poses, we can go to idle or score l3
            new PoseLink(SuperStructurePose.IDLE, SuperStructurePose.LOW_SWAP_1),
            new PoseLink(SuperStructurePose.IDLE, SuperStructurePose.LOW_SWAP_2),
            new PoseLink(SuperStructurePose.SCORE_L2, SuperStructurePose.LOW_SWAP_1),
            new PoseLink(SuperStructurePose.SCORE_L2, SuperStructurePose.LOW_SWAP_2),
            new PoseLink(SuperStructurePose.SCORE_L3, SuperStructurePose.LOW_SWAP_1),

            // From a few poses can we run to high swap
            new PoseLink(SuperStructurePose.IDLE, SuperStructurePose.HIGH_SWAP),
            new PoseLink(SuperStructurePose.SCORE_L3, SuperStructurePose.HIGH_SWAP),
            new PoseLink(SuperStructurePose.SCORE_L2, SuperStructurePose.HIGH_SWAP),
            new PoseLink(SuperStructurePose.LOW_SWAP_1, SuperStructurePose.HIGH_SWAP),

            // from high swap we can run to l4
            new PoseLink(SuperStructurePose.HIGH_SWAP, SuperStructurePose.SCORE_L4),

            // Legacy links (for dev bot)
            new PoseLink(SuperStructurePose.IDLE, SuperStructurePose.PREPARE_TO_RUN_UP_LEGACY),
            new PoseLink(SuperStructurePose.SCORE_L2, SuperStructurePose.PREPARE_TO_RUN_UP_LEGACY),
            new PoseLink(SuperStructurePose.LOW_SWAP_2, SuperStructurePose.HIGH_SWAP_LEGACY),
            new PoseLink(SuperStructurePose.PREPARE_TO_RUN_UP_LEGACY, SuperStructurePose.HIGH_SWAP_LEGACY),
            new PoseLink(SuperStructurePose.HIGH_SWAP_LEGACY, SuperStructurePose.SCORE_L4_LEGACY));

    /**
     * Represents a link between two super structure poses
     *
     * <p>If two poses are linked, it means that the super structure can move from pose1 to pose2 directly (without
     * hitting dead-bands).
     */
    public record PoseLink(SuperStructurePose pose1, SuperStructurePose pose2) {
        /** Calculates the amount of time needed for the super structure to move */
        public double timeSeconds() {
            // Differences between the current and target poses
            double armDifferenceRad = pose2.armAngle.minus(pose1.armAngle).abs(Radians);
            double elevatorDifferenceM =
                    pose2.elevatorHeight.minus(pose1.elevatorHeight).abs(Meters);

            // Constraints from the constants file
            double armMaxAcc = ArmConstants.PROFILE_CONSTRAINS.maxAcceleration;
            double armMaxVel = ArmConstants.PROFILE_CONSTRAINS.maxVelocity;
            double elevatorAcc = ElevatorConstants.PROFILE_CONSTRAINS.maxAcceleration;
            double elevatorVel = ElevatorConstants.PROFILE_CONSTRAINS.maxVelocity;

            // Time to move arm
            double armTime = calculateTimeToSetpoint(armDifferenceRad, armMaxAcc, armMaxVel);

            // Time to move elevator
            double elevatorTime = calculateTimeToSetpoint(elevatorDifferenceM, elevatorAcc, elevatorVel);

            // The total time is the maximum of the arm and elevator times
            return Math.max(armTime, elevatorTime);
        }

        /** Helper method to calculate time needed for a mechanism to move to a setpoint Author: ChatGPT-o3-mini-high */
        private static double calculateTimeToSetpoint(double difference, double maxAcc, double maxVel) {
            // Take the absolute value of difference to prevent negative values
            difference = Math.abs(difference);

            // Time to reach max velocity during acceleration phase
            double timeToMaxVel = maxVel / maxAcc;

            // Distance covered during acceleration to max velocity
            double distanceDuringAcc = 0.5 * maxAcc * timeToMaxVel * timeToMaxVel;

            // If the difference can be covered during acceleration and deceleration
            if (difference <= 2 * distanceDuringAcc)
                // We won't reach max velocity, so use the equation for constant acceleration/deceleration
                return 2 * Math.sqrt(difference / maxAcc);

            // Time to accelerate and decelerate
            double distanceAtConstantVelocity = difference - 2 * distanceDuringAcc;
            double timeAtConstantVel = distanceAtConstantVelocity / maxVel;

            return 2 * timeToMaxVel + timeAtConstantVel;
        }

        public Optional<SuperStructurePose> otherEdge(SuperStructurePose oneEdge) {
            if (oneEdge == pose1) return Optional.of(pose2);
            if (oneEdge == pose2) return Optional.of(pose1);
            else return Optional.empty();
        }
    }

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

        new Trigger(DriverStation::isDisabled)
                .onTrue(Commands.waitSeconds(2)
                        .andThen(() -> currentPose = SuperStructurePose.IDLE)
                        .until(DriverStation::isEnabled)
                        .ignoringDisable(true));
    }

    private Command runPose(SuperStructurePose pose) {
        return elevator.moveToPosition(pose.elevatorHeight)
                .alongWith(arm.moveToPosition(pose.armAngle))
                .beforeStarting(Commands.print("Super Structure/Moving to pose: " + pose.name()))
                .finallyDo(interrupted -> {
                    currentPose = pose;
                    if (interrupted)
                        System.out.println("Super Structure/Interrupted while running to pose: " + pose.name());
                    else System.out.println("Super Structure/Reached pose: " + pose.name());
                });
    }

    public Command moveToPose(SuperStructurePose pose) {
        return Commands.defer(() -> generateMoveToPoseCommand(pose), Set.of(elevator, arm));
    }

    private Command generateMoveToPoseCommand(SuperStructurePose pose) {
        Optional<List<SuperStructurePose>> trajectory = getTrajectory(pose);
        if (trajectory.isEmpty()) return Commands.none();
        return Commands.sequence(trajectory.get().stream().map(this::runPose).toArray(Command[]::new));
    }

    public SuperStructurePose currentPose() {
        return currentPose;
    }

    private static final int loopNumLimit = 100;
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
        PriorityQueue<PoseLink> linksToExamine = new PriorityQueue<>(Comparator.comparingDouble(PoseLink::timeSeconds));
        for (SuperStructurePose pose : SuperStructurePose.values())
            minimumTimeToPose.put(pose, Double.POSITIVE_INFINITY);
        minimumTimeToPose.put(startingPose, 0.0);

        SuperStructurePose currentNode = startingPose;

        int i;
        // limit loop time to avoid code crashes
        for (i = 0; i < loopNumLimit; i++) {
            for (PoseLink link : LINKS) {
                Optional<SuperStructurePose> otherNode = link.otherEdge(currentNode);
                if (otherNode.isEmpty()) continue;
                linksToExamine.add(link);

                double newTime = minimumTimeToPose.get(currentNode) + link.timeSeconds();
                if (newTime < minimumTimeToPose.get(otherNode.get())) {
                    minimumTimePathToNode.put(otherNode.get(), link);
                    minimumTimeToPose.put(otherNode.get(), newTime);
                }
            }

            unvisited.remove(currentNode);

            // select the next
            PoseLink linkToExamine;
            while ((linkToExamine = linksToExamine.poll()) != null) {
                if (unvisited.contains(linkToExamine.pose1)) {
                    currentNode = linkToExamine.pose1;
                    break;
                }
                if (unvisited.contains(linkToExamine.pose2)) {
                    currentNode = linkToExamine.pose2;
                    break;
                }
            }
            if (linkToExamine == null) break;
        }

        List<SuperStructurePose> trajectory = new ArrayList<>();
        SuperStructurePose tmp = targetPose;
        System.out.println("<-- tracing trajectory: -->");
        for (int j = 0; j < loopNumLimit; j++) {
            System.out.println("    tracing node: " + tmp);
            trajectory.add(0, tmp);
            if (tmp == startingPose) {
                System.out.println("Successfully planned trajectory: " + printTrajectory(trajectory));
                return Optional.of(trajectory);
            }
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

    private static String printTrajectory(List<SuperStructurePose> trajectory) {
        if (trajectory.isEmpty()) return "(Empty Trajectory)";
        StringBuilder message = new StringBuilder();
        for (int i = 0; i < trajectory.size() - 1; i++)
            message.append(trajectory.get(i).name()).append(" -> ");
        message.append(trajectory.get(trajectory.size() - 1).name());
        return message.toString();
    }

    public Optional<List<SuperStructurePose>> getTrajectory(SuperStructurePose targetPose) {
        return getTrajectory(currentPose, targetPose);
    }
}
