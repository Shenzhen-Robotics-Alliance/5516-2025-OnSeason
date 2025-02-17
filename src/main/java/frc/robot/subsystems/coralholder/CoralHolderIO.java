package frc.robot.subsystems.coralholder;

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
        public double rollerMotorCurrentAmps = 0.0;
        public double rollerMotorOutputVolts = 0.0;

        public double feederMotorCurrentAmps = 0.0;
        public double feederMotorOutputVolts = 0.0;
    }

    void updateInputs(CoralHolderInputs inputs);

    default void setRollerMotorOutput(double volts) {}

    default void setCollectorMotorOutput(double volts) {}
}
