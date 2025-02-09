package frc.robot.subsystems.superstructure.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    class ElevatorInputs {
        /** Whether the CAN communications between the rio and the motor are good. */
        public boolean hardwareConnected = false;

        /** The relative encoder angle, measured by the relative encoder. Gearing is NOT considered. */
        public Angle encoderAngle = Rotations.zero();

        /** The relative encoder velocity, measured by the relative encoder. Gearing is NOT considered. */
        public AngularVelocity encoderVelocity = RotationsPerSecond.zero();

        /** The supply current of the motor. */
        public Current motorSupplyCurrent = Amps.zero();

        /** The actual output voltage of the motor. */
        public Voltage motorOutputVoltage = Volts.zero();

        public Temperature motorTemperature = Celsius.of(24);
    }

    void updateInputs(ElevatorInputs inputs);

    default void setMotorOutput(Voltage voltage) {}

    default void setMotorBrake(boolean brakeModeEnable) {}
}
