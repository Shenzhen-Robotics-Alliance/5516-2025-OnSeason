package frc.robot.subsystems.coralholder;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public final class CoralHolderConstants {
    public static final Current ROLLERS_CURRENT_LIMIT = Amps.of(20);

    // Simulation Constants
    public static final Translation3d COLLECTOR_POSITION_ON_ROBOT = new Translation3d(-0.3, 0, 0.6);
    public static Translation3d COLLECTOR_RANGE = new Translation3d(0.2, 0.3, 0.2);

    public static final double COLLECTOR_TIME_SECONDS_AT_6V = 0.6;
    public static final double ROLLER_TIME_SECONDS_AT_6V = 1;
    public static final LinearVelocity CORAL_LAUNCHING_VELOCITY_6V = MetersPerSecond.of(1);

    public static final Distance CORAL_LENGTH_ON_ARM = Centimeters.of(35.2);
    public static final Rotation2d ARM_ANGLE_TO_CORAL_POINTING_ANGLE = Rotation2d.fromDegrees(-148);
    public static final Rotation2d ARM_PINPOINT_TO_CORAL_DIRECTION = Rotation2d.fromDegrees(-5);
}
