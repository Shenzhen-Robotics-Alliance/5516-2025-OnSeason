package frc.robot.subsystems.superstructure.arm;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

import static edu.wpi.first.units.Units.*;

public final class ArmConstants {
    public enum ArmPosition {
        IDLE(116),
        INTAKE(126),
        SCORE_L1_L2_L3(116),
        SCORE_L4(70),
        ELEVATOR_MOVING_UP(55);

        private final Angle angle;
        ArmPosition(double degrees) {
            this.angle = Degrees.of(degrees);
        }
    }

    // General Configs
    public static final double ARM_GEARING_REDUCTION = 48.0 / 12.0 * 48.0 / 20.0 * 9.0;
    public static final Current ARM_CURRENT_LIMIT = Amps.of(30.0);
    public static final Angle ARM_UPPER_LIMIT = Degrees.of(128);
    public static final Angle ARM_LOWER_LIMIT = Degrees.of(30);


    // PID Configuration
    public static final double kS = 0.05;
    public static final double kG = 0.17;
    public static final double kV = 1.2;
    public static final double kA = 0.01;
    public static final double kP_STRONG = 9.0/Math.toRadians(30);
    public static final double kP_WEAK = 3.0/Math.toRadians(30);
    public static final TrapezoidProfile.Constraints PROFILE_CONSTRAINS = new TrapezoidProfile.Constraints(
            Math.toRadians(360), Math.toRadians(500));
    public static final Angle ARM_PID_TOLERANCE = Degrees.of(4);

    // Simulation Constants
    public static final Distance ARM_LENGTH = Centimeters.of(25);
    public static final Mass ARM_MASS = Kilograms.of(8);
    public static final DCMotor ARM_GEARBOX = DCMotor.getKrakenX60(1);
}
