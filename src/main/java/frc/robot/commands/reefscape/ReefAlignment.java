package frc.robot.commands.reefscape;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Meters;
import static frc.robot.constants.ReefConstants.*;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.commands.drive.AutoAlignment;
import frc.robot.commands.drive.JoystickDriveAndAimAtTarget;
import frc.robot.constants.DriveControlLoops;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.drive.HolonomicDriveSubsystem;
import frc.robot.subsystems.led.LEDAnimation;
import frc.robot.subsystems.led.LEDStatusLight;
import frc.robot.subsystems.vision.apriltags.AprilTagVision;
import frc.robot.utils.MapleJoystickDriveInput;
import java.util.Arrays;
import java.util.Optional;
import java.util.OptionalInt;
import org.ironmaple.utils.FieldMirroringUtils;
import org.littletonrobotics.junction.Logger;

public class ReefAlignment {
    public static final Distance ROUGH_APPROACHT_POSE_TO_TARGET_DISTANCE = Meters.of(1.25);
    public static final Distance ROUGH_APPROACH_POSE_TO_TARGET_MARGIN = Centimeters.of(15);
    private static final Translation2d REEF_CENTER_BLUE = new Translation2d(4.5, 4);

    public record BranchTarget(
            Rotation2d facing,
            Translation2d roughApproachPosition,
            Translation2d preciseAlignmentPosition,
            int tagId,
            boolean rightSide) {

        public Pose2d roughApproachPose() {
            return new Pose2d(roughApproachPosition, facing);
        }

        public Pose2d preciseAlignmentPose() {
            return new Pose2d(preciseAlignmentPosition, facing);
        }

        public AutoAlignment.AutoAlignmentTarget autoAlignmentTarget() {
            return new AutoAlignment.AutoAlignmentTarget(
                    roughApproachPose(),
                    preciseAlignmentPose(),
                    facing(),
                    Optional.of(FieldMirroringUtils.toCurrentAllianceTranslation(REEF_CENTER_BLUE)),
                    OptionalInt.of(tagId),
                    rightSide ? 0 : 1,
                    rightSide ? 2 : 3);
        }

        public static BranchTarget measured(
                int tagID, boolean rightSide, Distance distanceToTag, Distance biasFromCenter) {
            Pose2d tagPose =
                    VisionConstants.fieldLayout.getTagPose(tagID).orElseThrow().toPose2d();
            Twist2d tagToTarget =
                    new Twist2d(distanceToTag.in(Meters), (rightSide ? 1 : -1) * biasFromCenter.in(Meters), 0);
            Twist2d tagToRoughTarget = new Twist2d(
                    ROUGH_APPROACHT_POSE_TO_TARGET_DISTANCE.in(Meters),
                    (rightSide ? 1 : -1) * ROUGH_APPROACH_POSE_TO_TARGET_MARGIN.in(Meters),
                    0);
            Translation2d targetPosition = tagPose.exp(tagToTarget).getTranslation();
            Translation2d roughTargetPosition = tagPose.exp(tagToRoughTarget).getTranslation();

            return new ReefAlignment.BranchTarget(
                    tagPose.getRotation().rotateBy(Rotation2d.k180deg),
                    roughTargetPosition,
                    targetPosition,
                    tagID,
                    rightSide);
        }
    }

    // 0 to 5
    private static int selectedReefPartId = 0;
    private static final Subsystem lock = new Subsystem() {};

    public static BranchTarget getSelectedReefAlignmentTarget(boolean rightSide) {
        int branchIndex = getBranchIndexFromReefPartId(rightSide);
        return FieldMirroringUtils.isSidePresentedAsRed()
                ? REEF_ALIGNMENT_POSITIONS_RED[branchIndex]
                : REEF_ALIGNMENT_POSITIONS_BLUE[branchIndex];
    }

    public static BranchTarget getNearestReefAlignmentTarget(Translation2d robotPosition, boolean rightSide) {
        BranchTarget minDistanceTarget = null;
        double minDistance = Double.POSITIVE_INFINITY;
        for (int i = 0; i < 12; i++) {
            BranchTarget target = FieldMirroringUtils.isSidePresentedAsRed()
                    ? REEF_ALIGNMENT_POSITIONS_RED[i]
                    : REEF_ALIGNMENT_POSITIONS_BLUE[i];
            double robotToTargetDistance =
                    target.roughApproachPosition().minus(robotPosition).getNorm();
            if (robotToTargetDistance > minDistance || rightSide != target.rightSide) continue;
            minDistance = robotToTargetDistance;
            minDistanceTarget = target;
        }
        return minDistanceTarget;
    }

    private static int getBranchIndexFromReefPartId(boolean rightSide) {
        int branchIndex = selectedReefPartId * 2;
        boolean isUpperSide = // selectedReefPartId == 2 || selectedReefPartId == 3 || selectedReefPartId == 4;
                false;
        if (rightSide ^ isUpperSide) branchIndex++;
        return branchIndex;
    }

    public static boolean[] displaySelectedBranch() {
        boolean[] reef = new boolean[12];
        Arrays.fill(reef, false);
        switch (selectedSide) {
            case LEFT -> reef[getBranchIndexFromReefPartId(false)] = true;
            case RIGHT -> reef[getBranchIndexFromReefPartId(true)] = true;
            case NOT_SELECTED -> reef[getBranchIndexFromReefPartId(false)] =
                    reef[getBranchIndexFromReefPartId(true)] = true;
        }
        return reef;
    }

    public static void nextTarget() {
        if (++selectedReefPartId >= 6) selectedReefPartId = 0;
    }

    public static void previousTarget() {
        if (--selectedReefPartId < 0) selectedReefPartId = 5;
    }

    public static void selectReefPart(int reefPartId) {
        selectedReefPartId = reefPartId;
    }

    public static Command selectReefPartButton(int reefPartId) {
        return Commands.runOnce(() -> selectReefPart(reefPartId), lock);
    }

    public static Command nextTargetButton(double debugTime) {
        return Commands.runOnce(ReefAlignment::nextTarget, lock)
                .andThen(Commands.waitSeconds(debugTime))
                .repeatedly();
    }

    public static Command previousTargetButton(double debugTime) {
        return Commands.runOnce(ReefAlignment::previousTarget, lock)
                .andThen(Commands.waitSeconds(debugTime))
                .repeatedly();
    }

    public static void lefterTarget() {
        if (selectedReefPartId == 0 || selectedReefPartId == 1 || selectedReefPartId == 5) previousTarget();
        else nextTarget();
    }

    public static void righterTarget() {
        if (selectedReefPartId == 0 || selectedReefPartId == 1 || selectedReefPartId == 5) nextTarget();
        else previousTarget();
    }

    public static Command lefterTargetButton(double debugTime) {
        return Commands.runOnce(ReefAlignment::lefterTarget, lock)
                .andThen(Commands.waitSeconds(debugTime))
                .repeatedly();
    }

    public static Command righterTargetButton(double debugTime) {
        return Commands.runOnce(ReefAlignment::righterTarget, lock)
                .andThen(Commands.waitSeconds(debugTime))
                .repeatedly();
    }

    public static Command followPathAndAlign(
            RobotContainer robot, PathPlannerPath path, int targetId, Command... toScheduleBeforePreciseAlignment) {
        return Commands.deferredProxy(() -> {
            BranchTarget branchTarget = (FieldMirroringUtils.isSidePresentedAsRed()
                            ? REEF_ALIGNMENT_POSITIONS_RED
                            : REEF_ALIGNMENT_POSITIONS_BLUE)
                    [targetId];
            return AutoAlignment.followPathAndAutoAlignStatic(
                            robot.drive,
                            robot.aprilTagVision,
                            robot.ledStatusLight,
                            path,
                            branchTarget.autoAlignmentTarget(),
                            DriveControlLoops.REEF_ALIGNMENT_CONFIG_AUTONOMOUS,
                            toScheduleBeforePreciseAlignment)
                    .beforeStarting(() -> {
                        selectedReefPartId = targetId / 2;
                        selectedSide = targetId % 2 == 0 ? SelectedSide.LEFT : SelectedSide.RIGHT;
                    })
                    .finallyDo(() -> selectedSide = SelectedSide.NOT_SELECTED);
        });
    }

    public static Command alignmentToSelectedBranch(
            HolonomicDriveSubsystem drive,
            AprilTagVision aprilTagVision,
            LEDStatusLight statusLight,
            boolean rightSide,
            Command... toScheduleAtPreciseAlignment) {
        return Commands.deferredProxy(() -> pathFindAndAlignToBranchStatic(
                        drive,
                        aprilTagVision,
                        statusLight,
                        getSelectedReefAlignmentTarget(rightSide),
                        toScheduleAtPreciseAlignment))
                .withName("[Reef Alignment] Align to branch " + getBranchIndexFromReefPartId(rightSide))
                .beforeStarting(() -> selectedSide = rightSide ? SelectedSide.RIGHT : SelectedSide.LEFT)
                .finallyDo(() -> selectedSide = SelectedSide.NOT_SELECTED);
    }

    private static final double AVERAGE_POSE_ESTIMATION_COUNT_THRESHOLD = 0.3;

    public static Command alignToNearestBranch(
            HolonomicDriveSubsystem drive,
            MapleJoystickDriveInput driveInput,
            AprilTagVision aprilTagVision,
            LEDStatusLight statusLight,
            boolean rightSide,
            Command... toScheduleAtPreciseAlignment) {
        Command faceToReefAndDrive = JoystickDriveAndAimAtTarget.driveAndAimAtTarget(
                driveInput,
                drive,
                () -> FieldMirroringUtils.toCurrentAllianceTranslation(REEF_CENTER_BLUE),
                null,
                0.8,
                false);

        Command waitingForVisionResultsLED =
                statusLight.playAnimationPeriodically(new LEDAnimation.Charging(Color.kHotPink), 1);
        Command visionReadyLED = statusLight.playAnimationPeriodically(new LEDAnimation.Breathe(Color.kHotPink), 1);

        Command waitUntilAlignmentReady = Commands.sequence(
                Commands.waitUntil(() ->
                                aprilTagVision.averagePoseEstimationsCount() > AVERAGE_POSE_ESTIMATION_COUNT_THRESHOLD)
                        .deadlineFor(waitingForVisionResultsLED),
                Commands.waitUntil(driveInput::isZeroInput).deadlineFor(visionReadyLED));
        return Commands.sequence(
                waitUntilAlignmentReady.deadlineFor(faceToReefAndDrive),
                Commands.deferredProxy(() -> pathFindAndAlignToBranchStatic(
                        drive,
                        aprilTagVision,
                        statusLight,
                        getNearestReefAlignmentTarget(
                                RobotState.getInstance().getVisionPose().getTranslation(), rightSide),
                        toScheduleAtPreciseAlignment)));
    }

    private static Command pathFindAndAlignToBranchStatic(
            HolonomicDriveSubsystem drive,
            AprilTagVision aprilTagVision,
            LEDStatusLight statusLight,
            BranchTarget branch,
            Command... toScheduleAtPreciseAlignment) {
        return AutoAlignment.pathFindAndAutoAlign(
                        drive,
                        aprilTagVision,
                        statusLight,
                        branch.autoAlignmentTarget(),
                        DriveControlLoops.REEF_ALIGNMENT_CONFIG,
                        toScheduleAtPreciseAlignment)
                .withName("[Reef Alignment] Align to branch");
    }

    private enum SelectedSide {
        LEFT,
        RIGHT,
        NOT_SELECTED
    }

    private static SelectedSide selectedSide = SelectedSide.NOT_SELECTED;

    public static void updateDashboard() {
        Logger.recordOutput("Reef/SelectedBranch", ReefAlignment.displaySelectedBranch());
        int selectedBranchTagID = getSelectedReefAlignmentTarget(false).tagId;
        Optional<Pose3d> tagPose3d = VisionConstants.fieldLayout.getTagPose(selectedBranchTagID);
        if (tagPose3d.isEmpty()) return;
        Pose2d tagRawPose = tagPose3d.get().toPose2d();
        Pose2d tagPose =
                new Pose2d(tagRawPose.getTranslation(), tagRawPose.getRotation().rotateBy(Rotation2d.k180deg));
        Logger.recordOutput(
                "Reef/RobotToSelectedBranchTag",
                RobotState.getInstance().getVisionPose().log(tagPose));
    }
}
