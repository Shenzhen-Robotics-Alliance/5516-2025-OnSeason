package frc.robot.subsystems.superstructure.arm;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.*;

public final class ArmConstants {
    // General Configs
    public static final double ARM_GEARING_REDUCTION = 44.0 / 16.0 * 9.0;
    public static final Current ARM_CURRENT_LIMIT = Amps.of(30.0);
    public static final Angle ARM_UPPER_LIMIT = Degrees.of(136);
    public static final Angle ARM_LOWER_LIMIT = Degrees.of(-48);
    public static final double ARM_MAX_VOLTS = 8;

    // PID Configuration
    public static final double kS = 0.03;
    public static final double kG = 0.31;
    public static final double kV = 0.49;
    public static final double kA = 0.01;
    public static final double kP = 9.0 / Math.toRadians(30);
    public static final TrapezoidProfile.Constraints PROFILE_CONSTRAINS =
            new TrapezoidProfile.Constraints(Math.toRadians(900), Math.toRadians(720));
    public static final Angle ARM_PID_TOLERANCE = Degrees.of(3);

    // Simulation Constants
    public static final Distance ARM_LENGTH = Centimeters.of(26);
    public static final Mass ARM_MASS = Kilograms.of(3);
    public static final DCMotor ARM_GEARBOX = DCMotor.getKrakenX60(1);
}
