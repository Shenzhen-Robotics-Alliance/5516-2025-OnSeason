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
        public boolean firstSensorTriggered = false;
        public boolean secondSensorTriggered = false;
        public boolean motorConnected = false;
        public Current rollerMotorCurrent = Amps.zero();
        public Voltage rollerMotorOutputVoltage = Volts.zero();
    }

    void updateInputs(CoralHolderInputs inputs);

    default void setRollerMotorOutput(Voltage voltage) {}
}
