package frc.robot.constants;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.commands.reefscape.ReefAlignment;
import java.util.ArrayList;
import java.util.List;

public class ReefConstants {
    private static final Distance ROUGH_APPROACHT_POSE_TO_TARGET_DISTANCE = Meters.of(1.5);
    private static final Distance ROUGH_APPROACH_POSE_TO_TARGET_MARGIN = Centimeters.of(15);

    public static void loadStatic() {
        ReefAlignment.BranchTarget target1 = REEF_ALIGNMENT_POSITIONS_BLUE[0];
        ReefAlignment.BranchTarget target2 = REEF_ALIGNMENT_POSITIONS_RED[0];
    }

    public static final ReefAlignment.BranchTarget[] REEF_ALIGNMENT_POSITIONS_BLUE = generateReefConstants(
            new ReefAlignment.BranchTarget(
                    Rotation2d.fromDegrees(0), new Translation2d(2.4, 4.0), new Translation2d(3.22, 4.18), 18, false),
            new ReefAlignment.BranchTarget(
                    Rotation2d.fromDegrees(0), new Translation2d(2.4, 4.0), new Translation2d(3.21, 3.87), 18, true),
            DriverStation.Alliance.Blue);
    //            new ReefAlignment.BranchTarget[] {
    //        new ReefAlignment.BranchTarget(
    //                Rotation2d.fromDegrees(0), new Translation2d(2.4, 4.0), new Translation2d(3.22, 4.18), 18, false),
    //        new ReefAlignment.BranchTarget(
    //                Rotation2d.fromDegrees(0), new Translation2d(2.4, 4.0), new Translation2d(3.21, 3.87), 18, true),
    //        new ReefAlignment.BranchTarget(
    //                Rotation2d.fromDegrees(60),
    //                new Translation2d(3.5, 2.3),
    //                new Translation2d(FIELD_WIDTH - 13.83, FIELD_HEIGHT - 5.06),
    //                17,
    //                false),
    //        new ReefAlignment.BranchTarget(
    //                Rotation2d.fromDegrees(60),
    //                new Translation2d(3.5, 2.3),
    //                new Translation2d(FIELD_WIDTH - 13.56, FIELD_HEIGHT - 5.22),
    //                17,
    //                true),
    //        new ReefAlignment.BranchTarget(
    //                Rotation2d.fromDegrees(120),
    //                new Translation2d(5.5, 2.3),
    //                new Translation2d(FIELD_WIDTH - 12.56, FIELD_HEIGHT - 5.21),
    //                22,
    //                false),
    //        new ReefAlignment.BranchTarget(
    //                Rotation2d.fromDegrees(120),
    //                new Translation2d(5.5, 2.3),
    //                new Translation2d(FIELD_WIDTH - 12.26, FIELD_HEIGHT - 5.04),
    //                22,
    //                true),
    //        new ReefAlignment.BranchTarget(
    //                Rotation2d.fromDegrees(180),
    //                new Translation2d(6.5, 4),
    //                new Translation2d(FIELD_WIDTH - 11.78, FIELD_HEIGHT - 4.17),
    //                21,
    //                false),
    //        new ReefAlignment.BranchTarget(
    //                Rotation2d.fromDegrees(180),
    //                new Translation2d(6.5, 4),
    //                new Translation2d(FIELD_WIDTH - 11.78, FIELD_HEIGHT - 3.86),
    //                21,
    //                true),
    //        new ReefAlignment.BranchTarget(
    //                Rotation2d.fromDegrees(-120), new Translation2d(5.4, 5.8), new Translation2d(5.24, 5.06), 20,
    // false),
    //        new ReefAlignment.BranchTarget(
    //                Rotation2d.fromDegrees(-120), new Translation2d(5.4, 5.8), new Translation2d(4.98, 5.21), 20,
    // true),
    //        new ReefAlignment.BranchTarget(
    //                Rotation2d.fromDegrees(-60), new Translation2d(3.6, 5.8), new Translation2d(3.99, 5.21), 19,
    // false),
    //        new ReefAlignment.BranchTarget(
    //                Rotation2d.fromDegrees(-60), new Translation2d(3.6, 5.8), new Translation2d(3.70, 5.04), 19, true)
    //    };

    public static final ReefAlignment.BranchTarget[] REEF_ALIGNMENT_POSITIONS_RED = generateReefConstants(
            new ReefAlignment.BranchTarget(
                    Rotation2d.fromDegrees(0), new Translation2d(2.4, 4.0), new Translation2d(3.22, 4.18), 18, false),
            new ReefAlignment.BranchTarget(
                    Rotation2d.fromDegrees(0), new Translation2d(2.4, 4.0), new Translation2d(3.21, 3.87), 18, true),
            DriverStation.Alliance.Red);
    //            new ReefAlignment.BranchTarget[] {
    //        new ReefAlignment.BranchTarget(
    //                Rotation2d.fromDegrees(180), new Translation2d(15, 3.8), new Translation2d(14.34, 3.87), 7,
    // false),
    //        new ReefAlignment.BranchTarget(
    //                Rotation2d.fromDegrees(180), new Translation2d(15, 4.2), new Translation2d(14.34, 4.20), 7, true),
    //        new ReefAlignment.BranchTarget(
    //                Rotation2d.fromDegrees(-120), new Translation2d(14.4, 5.9), new Translation2d(13.83, 5.06), 8,
    // false),
    //        new ReefAlignment.BranchTarget(
    //                Rotation2d.fromDegrees(-120), new Translation2d(14, 6.0), new Translation2d(13.56, 5.22), 8,
    // true),
    //        new ReefAlignment.BranchTarget(
    //                Rotation2d.fromDegrees(-60), new Translation2d(12.3, 6.0), new Translation2d(12.56, 5.21), 9,
    // false),
    //        new ReefAlignment.BranchTarget(
    //                Rotation2d.fromDegrees(-60), new Translation2d(11.8, 5.7), new Translation2d(12.26, 5.04), 9,
    // true),
    //        new ReefAlignment.BranchTarget(
    //                Rotation2d.fromDegrees(0), new Translation2d(11.1, 4.3), new Translation2d(11.78, 4.17), 10,
    // false),
    //        new ReefAlignment.BranchTarget(
    //                Rotation2d.fromDegrees(0), new Translation2d(11.1, 2.8), new Translation2d(11.78, 3.86), 10,
    // true),
    //        new ReefAlignment.BranchTarget(
    //                Rotation2d.fromDegrees(60), new Translation2d(11.8, 2.3), new Translation2d(12.30, 2.99), 11,
    // false),
    //        new ReefAlignment.BranchTarget(
    //                Rotation2d.fromDegrees(60), new Translation2d(12.3, 2.1), new Translation2d(12.56, 2.83), 11,
    // true),
    //        new ReefAlignment.BranchTarget(
    //                Rotation2d.fromDegrees(120), new Translation2d(13.9, 2.1), new Translation2d(13.56, 2.84), 6,
    // false),
    //        new ReefAlignment.BranchTarget(
    //                Rotation2d.fromDegrees(120), new Translation2d(14.3, 2.3), new Translation2d(13.85, 3.01), 6,
    // true)
    //    };

    public static ReefAlignment.BranchTarget[] generateReefConstants(
            ReefAlignment.BranchTarget measuredLeftTarget,
            ReefAlignment.BranchTarget measuredRightTarget,
            DriverStation.Alliance side) {
        int[] tagIDs = side == DriverStation.Alliance.Red
                ? new int[] {7, 8, 9, 10, 11, 6}
                : new int[] {18, 17, 22, 21, 20, 19};
        if (measuredLeftTarget.rightSide()
                || (!measuredRightTarget.rightSide())
                || (measuredLeftTarget.tagId() != measuredRightTarget.tagId()))
            throw new IllegalArgumentException("Left Right Targets does not match!");
        Pose2d referenceTagPose = VisionConstants.fieldLayout
                .getTagPose(measuredLeftTarget.tagId())
                .orElseThrow()
                .toPose2d();
        Twist2d tagToLeftTarget = referenceTagPose.log(measuredLeftTarget.preciseAlignmentPose());
        Twist2d tagToRightTarget = referenceTagPose.log(measuredRightTarget.preciseAlignmentPose());
        List<ReefAlignment.BranchTarget> generatedTargets = new ArrayList<>();
        for (int currentTagID : tagIDs) {
            Pose2d tagPose = VisionConstants.fieldLayout
                    .getTagPose(currentTagID)
                    .orElseThrow()
                    .toPose2d();
            Translation2d leftTargetPosition = tagPose.exp(tagToLeftTarget).getTranslation();
            Translation2d rightTargetPosition = tagPose.exp(tagToRightTarget).getTranslation();
            Rotation2d desiredRotation = tagPose.getRotation().rotateBy(Rotation2d.k180deg);
            Translation2d leftRoughTarget = tagPose.exp(new Twist2d(
                            ROUGH_APPROACHT_POSE_TO_TARGET_DISTANCE.in(Meters),
                            -ROUGH_APPROACH_POSE_TO_TARGET_MARGIN.in(Meters),
                            0))
                    .getTranslation();
            Translation2d rightRoughTarget = tagPose.exp(new Twist2d(
                            ROUGH_APPROACHT_POSE_TO_TARGET_DISTANCE.in(Meters),
                            ROUGH_APPROACH_POSE_TO_TARGET_MARGIN.in(Meters),
                            0))
                    .getTranslation();

            generatedTargets.add(new ReefAlignment.BranchTarget(
                    desiredRotation, leftRoughTarget, leftTargetPosition, currentTagID, false));
            generatedTargets.add(new ReefAlignment.BranchTarget(
                    desiredRotation, rightRoughTarget, rightTargetPosition, currentTagID, true));
        }

        return generatedTargets.toArray(new ReefAlignment.BranchTarget[12]);
    }
}
