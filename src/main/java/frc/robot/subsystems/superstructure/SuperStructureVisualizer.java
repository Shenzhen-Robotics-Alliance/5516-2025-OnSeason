package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.coralholder.CoralHolderConstants.*;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.measure.Distance;
import org.littletonrobotics.junction.Logger;

public class SuperStructureVisualizer {
    private static final Translation3d ELEVATOR_ZERO_POSITION = new Translation3d(0, 0, 0.05);
    private static final Translation3d ARM_ZERO_POSITION = new Translation3d(0.265, 0, 0.37);
    private static final double ARM_ZERO_PITCH_RAD = Math.toRadians(130);

    public static void visualizeMechanisms(String key, double elevatorHeightMeters, Rotation2d armAngle) {
        Translation3d elevatorHeightTranslation = new Translation3d(0, 0, elevatorHeightMeters);
        Rotation3d armRotation = new Rotation3d(0, ARM_ZERO_PITCH_RAD - armAngle.getRadians(), 0);
        Pose3d[] poses = new Pose3d[] {
            // First stage elevator only ascends half the height.
            new Pose3d(elevatorHeightTranslation.plus(ELEVATOR_ZERO_POSITION).div(2), new Rotation3d()),
            // Carriage ascends full height.
            new Pose3d(elevatorHeightTranslation.plus(ELEVATOR_ZERO_POSITION), new Rotation3d()),
            // Arm ascends full height and rotates an angle
            new Pose3d(ARM_ZERO_POSITION.plus(ELEVATOR_ZERO_POSITION).plus(elevatorHeightTranslation), armRotation)
        };
        Logger.recordOutput("SuperStructure/" + key, poses);
    }

    public static void visualizeCoralInCoralHolder(
            String key,
            Pose2d robotPose,
            double elevatorHeightMeters,
            Rotation2d armAngle,
            Distance coralDisplacement) {
        Logger.recordOutput(key, getCoralOnRobotPosition(robotPose, elevatorHeightMeters, armAngle, coralDisplacement));
    }

    public static Pose3d getCoralOnRobotPosition(
            Pose2d robotPose, double elevatorHeightMeters, Rotation2d armAngle, Distance coralDisplacement) {
        Pose3d coralPoseOnRobot = getCoralPositionRobotRelative(elevatorHeightMeters, armAngle, coralDisplacement);

        Rotation3d coralOrientation = coralPoseOnRobot
                .getRotation()
                .plus(new Rotation3d(0, 0, robotPose.getRotation().getRadians()));

        Translation3d robotPosition = new Translation3d(robotPose.getTranslation());
        Translation3d coralPosition =
                robotPosition.plus(coralPoseOnRobot.getTranslation().rotateBy(new Rotation3d(robotPose.getRotation())));
        return new Pose3d(coralPosition, coralOrientation);
    }

    public static Pose3d getCoralPositionRobotRelative(
            double elevatorHeightMeters, Rotation2d armAngle, Distance coralDisplacement) {
        Rotation3d coralOrientationOnRobot = new Rotation3d(
                0, -armAngle.plus(ARM_ANGLE_TO_CORAL_POINTING_ANGLE).getRadians(), 0);
        Translation3d coralPositionOnRobot = new Translation3d(0.27, 0, 0.365)
                .plus(new Translation3d(
                        CORAL_LENGTH_ON_ARM.in(Meters),
                        new Rotation3d(
                                0,
                                -armAngle.plus(ARM_PINPOINT_TO_CORAL_DIRECTION).getRadians(),
                                0)))
                .plus(new Translation3d(0, 0, elevatorHeightMeters).plus(ELEVATOR_ZERO_POSITION))
                .plus(new Translation3d(coralDisplacement.in(Meters), coralOrientationOnRobot));

        return new Pose3d(coralPositionOnRobot, coralOrientationOnRobot);
    }
}
