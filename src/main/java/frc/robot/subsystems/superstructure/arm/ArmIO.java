package frc.robot.subsystems.superstructure.arm;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import java.util.Optional;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface ArmIO {
    final class ArmInputs implements LoggableInputs {
        /**
         * The (optional) arm absolute encoder angle, already calibrated. Empty if the Absolute Encoder is disconnected.
         */
        public Optional<Rotation2d> absoluteEncoderAngle;

        /** Whether the CAN communications between the rio and the motor are good. */
        public boolean motorConnected;

        /** The relative encoder angle, measured by the relative encoder. Gearing is NOT considered. */
        public Angle relativeEncoderAngle;

        /** The relative encoder velocity, measured by the relative encoder. Gearing is NOT considered. */
        public AngularVelocity encoderVelocity;

        /** The supply current of the motor. */
        public Current motorSupplyCurrent;

        /** The actual output voltage of the motor. */
        public Voltage motorOutputVoltage;

        public ArmInputs() {
            this.absoluteEncoderAngle = Optional.empty();
            motorConnected = false;
            this.relativeEncoderAngle = Rotations.zero();
            this.encoderVelocity = RotationsPerSecond.zero();
            this.motorSupplyCurrent = Amps.zero();
            this.motorOutputVoltage = Volts.zero();
        }

        @Override
        public void toLog(LogTable table) {
            table.put("absoluteEncoderAnglePresent", absoluteEncoderAngle.isPresent());
            table.put("absoluteEncoderAngle", absoluteEncoderAngle.orElse(Rotation2d.kZero));
            table.put("motorConnected", motorConnected);
            table.put("relativeEncoderAngle", relativeEncoderAngle);
            table.put("encoderVelocity", encoderVelocity);
            table.put("motorSupplyCurrent", motorSupplyCurrent);
            table.put("motorOutputVoltage", motorOutputVoltage);
        }

        @Override
        public void fromLog(LogTable table) {
            boolean absoluteEncoderAnglePresent = table.get("absoluteEncoderAnglePresent", false);
            absoluteEncoderAngle = absoluteEncoderAnglePresent
                    ? Optional.of(table.get("absoluteEncoderAngle", Rotation2d.kZero))
                    : Optional.empty();
            motorConnected = table.get("motorConnected", motorConnected);
            relativeEncoderAngle = table.get("relativeEncoderAngle", Rotations.zero());
            encoderVelocity = table.get("encoderVelocity", encoderVelocity);
            motorSupplyCurrent = table.get("motorSupplyCurrent", Amps.zero());
            motorOutputVoltage = table.get("motorOutputVoltage", Volts.zero());
        }
    }

    void updateInputs(ArmInputs armInputs);

    default void setMotorOutput(Voltage voltage) {}

    default void setMotorBrake(boolean brakeModeEnable) {}
}
