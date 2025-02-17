package frc.robot.subsystems.superstructure.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    class ElevatorInputs {
        /** Whether the CAN communications between the rio and the motor are good. */
        public boolean hardwareConnected = false;

        /** The relative encoder angle, measured by the relative encoder. Gearing is NOT considered. */
        public double encoderAngleRad = 0.0;

        /** The relative encoder velocity, measured by the relative encoder. Gearing is NOT considered. */
        public double encoderVelocityRadPerSec = 0.0;

        /** The supply current of the motor. */
        public double motorSupplyCurrentAmps = 0.0;

        /** The actual output voltage of the motor. */
        public double motorOutputVolts = 0.0;

        public double motorTemperatureCelsius = 0.0;
    }

    void updateInputs(ElevatorInputs inputs);

    default void setMotorOutput(double volts) {}

    default void setMotorBrake(boolean brakeModeEnable) {}
}
