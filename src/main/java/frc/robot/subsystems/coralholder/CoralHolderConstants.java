package frc.robot.subsystems.coralholder;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.Robot;

public final class CoralHolderConstants {
    public record HardwareConstants(
            int rollerMotorID,
            boolean rollerMotorInverted,
            int firstSensorID,
            int secondSensorID,
            int[] feederMotorsIDs,
            boolean[] feederMotorsInverted) {}

    public static final HardwareConstants HARDWARE_CONSTANTS =
            switch (Robot.CURRENT_ROBOT) {
                case TEAM_5516_DEVBOT_HYDROXIDE_I -> new HardwareConstants(
                        3, true, 0, 1, new int[] {5, 6}, new boolean[] {true, false});
                case TEAM_5516_COMPBOT_HYDROXIDE_II -> new HardwareConstants(
                        1, false, 1, 0, new int[] {5, 6}, new boolean[] {false, true});
                case TEAM_6706_COMPBOT -> new HardwareConstants(18, true, 0, 1, new int[0], new boolean[0]);
            };

    public record VoltageSettings(double INTAKE_VOLTS, double SHOOT_VOLTS, double BRAKE_VOLTS, double SHUFFLE_VOLTS) {}

    public static final VoltageSettings VOLTAGE_SETTINGS =
            switch (Robot.CURRENT_ROBOT) {
                case TEAM_5516_DEVBOT_HYDROXIDE_I, TEAM_5516_COMPBOT_HYDROXIDE_II -> new VoltageSettings(
                        3.5, 8.0, -1.5, 2.0);
                case TEAM_6706_COMPBOT -> new VoltageSettings(5.0, 8.0, -1.0, 1.5);
            };

    public static final Distance FIRST_SENSOR_THRESHOLD = Centimeters.of(3);
    public static final Distance SECOND_SENSOR_THRESHOLD = Centimeters.of(5);
    public static final Current ROLLERS_CURRENT_LIMIT = Amps.of(20);

    // Simulation Constants
    public static final Translation3d COLLECTOR_POSITION_ON_ROBOT = new Translation3d(-0.3, 0, 0.6);
    public static Translation3d COLLECTOR_RANGE = new Translation3d(0.2, 0.3, 0.2);

    public static final double COLLECTOR_TIME_SECONDS_AT_6V = 0.1;
    public static final double ROLLER_TIME_SECONDS_AT_6V = 0.2;
    public static final LinearVelocity CORAL_LAUNCHING_VELOCITY_6V = MetersPerSecond.of(3);

    public static final Distance CORAL_LENGTH_ON_ARM = Centimeters.of(35.2);
    public static final Rotation2d ARM_ANGLE_TO_CORAL_POINTING_ANGLE = Rotation2d.fromDegrees(-148);
    public static final Rotation2d ARM_PINPOINT_TO_CORAL_DIRECTION = Rotation2d.fromDegrees(-5);
}
