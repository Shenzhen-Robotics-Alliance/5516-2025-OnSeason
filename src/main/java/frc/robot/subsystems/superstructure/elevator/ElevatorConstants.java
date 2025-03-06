package frc.robot.subsystems.superstructure.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import frc.robot.Robot;

public final class ElevatorConstants {
    // General Constants (shared across all robots)
    public static final Distance HEIGHT_THRESHOLD_ENABLE_LOW_SPEED_MODE = Centimeters.of(45);
    public static final LinearVelocity ELEVATOR_MOVING_VELOCITY_THRESHOLD = MetersPerSecond.of(0.03);

    // Current Limits (shared across all robots)
    public static final Current STATOR_CURRENT_LIMIT = Amps.of(80);
    public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(60);
    public static final Time OVERHEAT_PROTECTION_TIME = Seconds.of(1);
    public static final Current OVERHEAT_PROTECTION_CURRENT = Amps.of(40);

    // Elevator Hardware Constants
    public record ElevatorHardwareConstants(
            Distance CHAIN_LENGTH,
            int ELEVATOR_DRUM_WHEEL_TEETH,
            int ELEVATOR_STAGES,
            double ELEVATOR_GEARING_REDUCTION,
            DCMotor ELEVATOR_GEARBOX,
            Mass ELEVATOR_CARRIAGE_WEIGHT,
            Distance ELEVATOR_MAX_HEIGHT,
            int ELEVATOR_MOTOR_ID,
            boolean ELEVATOR_MOTOR_INVERTED) {}

    public static final ElevatorHardwareConstants HARDWARE_CONSTANTS =
            switch (Robot.CURRENT_ROBOT) {
                case TEAM_5516_DEVBOT_HYDROXIDE_I -> new ElevatorHardwareConstants(
                        Inches.of(0.25),
                        22,
                        2,
                        5.0 * 60.0 / 36.0,
                        DCMotor.getKrakenX60(1),
                        Kilograms.of(10),
                        Meters.of(1.34),
                        1,
                        false);
                case TEAM_5516_COMPBOT_HYDROXIDE_II -> new ElevatorHardwareConstants(
                        Inches.of(0.25),
                        22,
                        2,
                        5.0 * 60.0 / 36.0,
                        DCMotor.getKrakenX60(1),
                        Kilograms.of(6),
                        Meters.of(1.34),
                        2,
                        false);
                    // TODO: constants for 6706 robot
                case TEAM_6706_COMPBOT -> new ElevatorHardwareConstants(
                        Inches.of(0.25),
                        22,
                        3,
                        5.0 * 60.0 / 36.0,
                        DCMotor.getKrakenX60(1),
                        Kilograms.of(6),
                        Meters.of(1.34),
                        1,
                        false);
            };

    // Elevator PID Constants
    public record ElevatorPIDConstants(
            double kS,
            double kG,
            double kV,
            double kA,
            double kP_STRONG,
            double kP_WEAK,
            Voltage MAX_OUTPUT_VOLTAGE,
            Voltage MIN_OUTPUT_VOLTAGE,
            LinearVelocity VELOCITY_CONSTRAIN,
            LinearAcceleration ACCELERATION_CONSTRAIN,
            Distance TOLERANCE) {}

    public static final ElevatorPIDConstants PID_CONSTANTS =
            switch (Robot.CURRENT_ROBOT) {
                case TEAM_5516_DEVBOT_HYDROXIDE_I -> new ElevatorPIDConstants(
                        0.1,
                        0.60,
                        3.70,
                        0.08,
                        7.5 / 0.2,
                        3.0 / 0.2,
                        Volts.of(12),
                        Volts.of(-8),
                        MetersPerSecond.of(2.4),
                        MetersPerSecondPerSecond.of(7.2),
                        Centimeters.of(2));
                case TEAM_5516_COMPBOT_HYDROXIDE_II -> new ElevatorPIDConstants(
                        0.1,
                        0.36,
                        3.70,
                        0.05,
                        7.5 / 0.2,
                        3.0 / 0.2,
                        Volts.of(12),
                        Volts.of(-8),
                        MetersPerSecond.of(2.4),
                        MetersPerSecondPerSecond.of(7.2),
                        Centimeters.of(2));
                case TEAM_6706_COMPBOT -> new ElevatorPIDConstants(
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0 / 0.2,
                        0.0 / 0.2,
                        Volts.of(12),
                        Volts.of(-8),
                        MetersPerSecond.of(1),
                        MetersPerSecondPerSecond.of(15),
                        Centimeters.of(2));
            };
}
