package frc.robot.subsystems.superstructure.arm;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import frc.robot.Robot;

public final class ArmConstants {
    public static final Current ARM_CURRENT_LIMIT = Amps.of(30.0);
    public static final Voltage ARM_MAX_VOLTAGE = Volts.of(8.0);

    public record ArmHardwareConstants(
            Distance ARM_COM_LENGTH,
            Mass ARM_MASS,
            DCMotor ARM_GEARBOX,
            double ARM_GEARING_REDUCTION,
            Angle ARM_UPPER_HARD_LIMIT,
            Angle ARM_LOWER_HARD_LIMIT,
            Angle ABSOLUTE_ENCODER_READING_AT_UPPER_LIM,
            int ABSOLUTE_ENCODER_CHANNEL,
            boolean ABSOLUTE_ENCODER_INVERTED,
            int ARM_MOTOR_ID,
            boolean ARM_MOTOR_INVERTED) {}

    public static final ArmHardwareConstants HARDWARE_CONSTANTS =
            switch (Robot.CURRENT_ROBOT) {
                case TEAM_5516_DEVBOT_HYDROXIDE_I -> new ArmHardwareConstants(
                        Centimeters.of(26.0),
                        Kilograms.of(4.0),
                        DCMotor.getKrakenX60(1),
                        44.0 / 16.0 * 48.0 / 20.0 * 9.0,
                        Degrees.of(136.0),
                        Degrees.of(-48.0),
                        Rotations.of(0.0),
                        0,
                        false,
                        4,
                        false);
                case TEAM_5516_COMPBOT_HYDROXIDE_II -> new ArmHardwareConstants(
                        Centimeters.of(26.0),
                        Kilograms.of(3.0),
                        DCMotor.getKrakenX60(1),
                        44.0 / 16.0 * 9.0,
                        Degrees.of(136.0),
                        Degrees.of(-48.0),
                        Rotations.of(0.260),
                        0,
                        false,
                        4,
                        true);
                case TEAM_6706_COMPBOT -> new ArmHardwareConstants(
                        Centimeters.of(26.0),
                        Kilograms.of(4.0),
                        DCMotor.getKrakenX60(1),
                        24.0 / 12.0 * 20.0,
                        Degrees.of(136.0),
                        Degrees.of(-48.0),
                        Rotations.of(0.10),
                        0,
                        true,
                        19,
                        false);
            };

    public record ArmPIDConstants(
            double kS,
            double kG,
            double kV,
            double kA,
            double kP,
            AngularVelocity VELOCITY_CONSTRAIN,
            AngularAcceleration ACCELERATION_CONSTRAIN,
            Angle TOLERANCE) {}

    public static final ArmPIDConstants PID_CONSTANTS =
            switch (Robot.CURRENT_ROBOT) {
                case TEAM_5516_DEVBOT_HYDROXIDE_I -> new ArmPIDConstants(
                        0.05,
                        0.3,
                        1.13,
                        0.01,
                        9.0 / Math.toRadians(30),
                        RotationsPerSecond.of(1.8),
                        RotationsPerSecondPerSecond.of(2.5),
                        Degrees.of(3));
                case TEAM_5516_COMPBOT_HYDROXIDE_II -> new ArmPIDConstants(
                        0.03,
                        0.31,
                        0.49,
                        0.01,
                        6.0 / Math.toRadians(30),
                        RotationsPerSecond.of(1.8),
                        RotationsPerSecondPerSecond.of(2.5),
                        Degrees.of(3));
                case TEAM_6706_COMPBOT -> new ArmPIDConstants(
                        0.05,
                        0.33,
                        0.76,
                        0.01,
                        0.0,
                        RotationsPerSecond.of(1),
                        RotationsPerSecondPerSecond.of(5),
                        Degrees.of(3));
            };
}
