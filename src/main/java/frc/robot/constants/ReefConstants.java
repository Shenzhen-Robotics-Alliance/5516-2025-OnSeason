package frc.robot.constants;

import static org.ironmaple.utils.FieldMirroringUtils.FIELD_HEIGHT;
import static org.ironmaple.utils.FieldMirroringUtils.FIELD_WIDTH;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.commands.reefscape.ReefAlignment;

public class ReefConstants {
    public static final ReefAlignment.BranchTarget[] REEF_ALIGNMENT_POSITIONS_BLUE = new ReefAlignment.BranchTarget[] {
        new ReefAlignment.BranchTarget(
                Rotation2d.fromDegrees(0), new Translation2d(2.4, 4.0), new Translation2d(3.22, 4.18), 18, false),
        new ReefAlignment.BranchTarget(
                Rotation2d.fromDegrees(0), new Translation2d(2.4, 4.0), new Translation2d(3.21, 3.87), 18, true),
        new ReefAlignment.BranchTarget(
                Rotation2d.fromDegrees(60),
                new Translation2d(3.5, 2.3),
                new Translation2d(FIELD_WIDTH - 13.83, FIELD_HEIGHT - 5.06),
                17,
                false),
        new ReefAlignment.BranchTarget(
                Rotation2d.fromDegrees(60),
                new Translation2d(3.5, 2.3),
                new Translation2d(FIELD_WIDTH - 13.56, FIELD_HEIGHT - 5.22),
                17,
                true),
        new ReefAlignment.BranchTarget(
                Rotation2d.fromDegrees(120),
                new Translation2d(5.5, 2.3),
                new Translation2d(FIELD_WIDTH - 12.56, FIELD_HEIGHT - 5.21),
                22,
                false),
        new ReefAlignment.BranchTarget(
                Rotation2d.fromDegrees(120),
                new Translation2d(5.5, 2.3),
                new Translation2d(FIELD_WIDTH - 12.26, FIELD_HEIGHT - 5.04),
                22,
                true),
        new ReefAlignment.BranchTarget(
                Rotation2d.fromDegrees(180),
                new Translation2d(6.5, 4),
                new Translation2d(FIELD_WIDTH - 11.78, FIELD_HEIGHT - 4.17),
                21,
                false),
        new ReefAlignment.BranchTarget(
                Rotation2d.fromDegrees(180),
                new Translation2d(6.5, 4),
                new Translation2d(FIELD_WIDTH - 11.78, FIELD_HEIGHT - 3.86),
                21,
                true),
        new ReefAlignment.BranchTarget(
                Rotation2d.fromDegrees(-120), new Translation2d(5.4, 5.8), new Translation2d(5.24, 5.06), 20, false),
        new ReefAlignment.BranchTarget(
                Rotation2d.fromDegrees(-120), new Translation2d(5.4, 5.8), new Translation2d(4.98, 5.21), 20, true),
        new ReefAlignment.BranchTarget(
                Rotation2d.fromDegrees(-60), new Translation2d(3.6, 5.8), new Translation2d(3.99, 5.21), 19, false),
        new ReefAlignment.BranchTarget(
                Rotation2d.fromDegrees(-60), new Translation2d(3.6, 5.8), new Translation2d(3.70, 5.04), 19, true)
    };

    public static final ReefAlignment.BranchTarget[] REEF_ALIGNMENT_POSITIONS_RED = new ReefAlignment.BranchTarget[] {
        new ReefAlignment.BranchTarget(
                Rotation2d.fromDegrees(180), new Translation2d(15, 3.8), new Translation2d(14.34, 3.87), 7, false),
        new ReefAlignment.BranchTarget(
                Rotation2d.fromDegrees(180), new Translation2d(15, 4.2), new Translation2d(14.34, 4.20), 7, true),
        new ReefAlignment.BranchTarget(
                Rotation2d.fromDegrees(-120), new Translation2d(14.4, 5.9), new Translation2d(13.83, 5.06), 8, false),
        new ReefAlignment.BranchTarget(
                Rotation2d.fromDegrees(-120), new Translation2d(14, 6.0), new Translation2d(13.56, 5.22), 8, true),
        new ReefAlignment.BranchTarget(
                Rotation2d.fromDegrees(-60), new Translation2d(12.3, 6.0), new Translation2d(12.56, 5.21), 9, false),
        new ReefAlignment.BranchTarget(
                Rotation2d.fromDegrees(-60), new Translation2d(11.8, 5.7), new Translation2d(12.26, 5.04), 9, true),
        new ReefAlignment.BranchTarget(
                Rotation2d.fromDegrees(0), new Translation2d(11.1, 4.3), new Translation2d(11.78, 4.17), 10, false),
        new ReefAlignment.BranchTarget(
                Rotation2d.fromDegrees(0), new Translation2d(11.1, 2.8), new Translation2d(11.78, 3.86), 10, true),
        new ReefAlignment.BranchTarget(
                Rotation2d.fromDegrees(60), new Translation2d(11.8, 2.3), new Translation2d(12.30, 2.99), 11, false),
        new ReefAlignment.BranchTarget(
                Rotation2d.fromDegrees(60), new Translation2d(12.3, 2.1), new Translation2d(12.56, 2.83), 11, true),
        new ReefAlignment.BranchTarget(
                Rotation2d.fromDegrees(120), new Translation2d(13.9, 2.1), new Translation2d(13.56, 2.84), 6, false),
        new ReefAlignment.BranchTarget(
                Rotation2d.fromDegrees(120), new Translation2d(14.3, 2.3), new Translation2d(13.85, 3.01), 6, true)
    };
}
