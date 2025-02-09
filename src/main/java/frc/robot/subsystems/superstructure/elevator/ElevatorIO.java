package frc.robot.subsystems.superstructure.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
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
    }

    void updateInputs(ElevatorInputs inputs);

    default void setMotorOutput(Voltage voltage) {}

    default void setMotorBrake(boolean brakeModeEnable) {}
}
