package frc.robot.subsystems.coralholder;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;

import static edu.wpi.first.units.Units.*;

public final class CoralHolderConstants {
    public static final Current ROLLERS_CURRENT_LIMIT = Amps.of(20);
    public static final Distance SENSOR_DISTANCE_THRESHOLD = Centimeters.of(5);

    // Simulation Constants
    public static final DCMotor ROLLER_GEARBOX = DCMotor.getFalcon500(1);
    public static final double ROLLER_GEARING_REDUCTION = 36.0/18.0;
    public static final Distance ROLLER_RADIUS = Centimeters.of(5.2/2.0);
}
