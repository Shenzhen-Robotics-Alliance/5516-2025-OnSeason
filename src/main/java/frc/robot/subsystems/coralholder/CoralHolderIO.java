package frc.robot.subsystems.coralholder;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface CoralHolderIO {
    @AutoLog
    class CoralHolderInputs {
        public boolean firstSensorConnected = false;
        public boolean secondSensorConnected = false;
        public boolean firstSensorReadingValid = false;
        public boolean secondSensorReadingValid = false;
        public double firstSensorDistanceMM = 0.0;
        public double secondSensorDistanceMM = 0.0;

        public boolean motorConnected = false;
        public Current rollerMotorCurrent = Amps.zero();
        public Voltage rollerMotorOutputVoltage = Volts.zero();

        public Current feederMotorCurrent = Amps.zero();
        public Voltage feederMotorOutputVoltage = Volts.zero();
    }

    void updateInputs(CoralHolderInputs inputs);

    default void setRollerMotorOutput(Voltage voltage) {}

    default void setCollectorMotorOutput(Voltage voltage) {}
}
