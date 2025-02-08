package frc.robot.subsystems.superstructure.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.*;

public final class ElevatorConstants {
    // General Constants
    public static final Distance CHAN_LENGTH = Inches.of(0.25);
    public static final int ELEVATOR_DRUM_WHEEL_TEETH = 22;
    public static final int ELEVATOR_STAGES = 2;
    public static final double ELEVATOR_GEARING_REDUCTION = 84.0 / 12.0;
    public static final Distance ELEVATOR_MAX_HEIGHT = Meters.of(1.34);

    // PID Constants
    public static final double kS = 0.03;
    public static final double kG = 0.72;
    public static final double kV = 2.83;
    public static final double kA = 0.09;
    public static final double kP_STRONG = 7.5 / 0.2;
    public static final double kP_WEAK = 3.0 / 0.2;
    public static final Voltage MAX_OUTPUT_VOLTAGE = Volts.of(12);
    public static final Voltage MIN_OUTPUT_VOLTAGE = Volts.of(-8);
    public static final TrapezoidProfile.Constraints PROFILE_CONSTRAINS = new TrapezoidProfile.Constraints(1.8, 4);
    public static final Current CURRENT_LIMIT = Amps.of(100);
    public static final Distance ELEVATOR_PID_TOLERANCE = Centimeters.of(4);
    public static final LinearVelocity ELEVATOR_MOVING_VELOCITY_THRESHOLD = MetersPerSecond.of(0.05);

    // Simulation Constants
    public static final DCMotor ELEVATOR_GEARBOX = DCMotor.getFalcon500(2);
    public static final Mass ELEVATOR_CARRIAGE_WEIGHT = Kilograms.of(10);
}
