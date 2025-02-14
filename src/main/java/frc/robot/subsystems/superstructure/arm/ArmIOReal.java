package frc.robot.subsystems.superstructure.arm;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.superstructure.arm.ArmConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Optional;

public class ArmIOReal implements ArmIO {
    /**
     * The absolute encoder reading when the arm hits the upper limit. Measure this on "SmartDashboard/Arm/Raw Encoder
     * Reading", which is in rotations.
     */
    private static final Angle ABSOLUTE_ENCODER_READING_AT_UPPER_LIMIT = Degrees.of(96);
    /**
     * The difference between the raw encoder reading angle and actual arm angle. Real Angle = Encoder Angle - Offset
     * Angle. Offset Angle = Encoder Angle - Real Angle.
     */
    private static final Rotation2d ABSOLUTE_ENCODER_OFFSET =
            new Rotation2d(ABSOLUTE_ENCODER_READING_AT_UPPER_LIMIT).minus(new Rotation2d(ARM_UPPER_LIMIT));

    // Hardware
    private final TalonFX armTalon;
    private final DutyCycleEncoder absoluteEncoder;

    // CTRE Motor Signals
    private final StatusSignal<Angle> relativeEncoderAngle;
    private final StatusSignal<AngularVelocity> relativeEncoderVelocity;
    private final StatusSignal<Current> motorSupplyCurrent;
    private final StatusSignal<Voltage> motorOutputVoltage;

    public ArmIOReal() {
        // Hardware Creation
        this.armTalon = new TalonFX(4);
        this.absoluteEncoder = new DutyCycleEncoder(0);

        // Configure Motor
        armTalon.getConfigurator()
                .apply(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));
        armTalon.getConfigurator()
                .apply(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimitEnable(true)
                        .withSupplyCurrentLimit(ARM_CURRENT_LIMIT));

        // Obtain Status Signals
        this.relativeEncoderAngle = armTalon.getPosition();
        this.relativeEncoderVelocity = armTalon.getVelocity();
        this.motorSupplyCurrent = armTalon.getSupplyCurrent();
        this.motorOutputVoltage = armTalon.getMotorVoltage();
        BaseStatusSignal.setUpdateFrequencyForAll(
                100.0, relativeEncoderAngle, relativeEncoderVelocity, motorSupplyCurrent, motorOutputVoltage);
        armTalon.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ArmInputs inputs) {
        // Obtain absolute encoder readings
        inputs.absoluteEncoderAngle = absoluteEncoder.isConnected()
                ? Optional.of(Rotation2d.fromRotations(absoluteEncoder.get()).minus(ABSOLUTE_ENCODER_OFFSET))
                : Optional.empty();

        // Refresh signals
        StatusCode statusCode = BaseStatusSignal.refreshAll(
                relativeEncoderAngle, relativeEncoderVelocity, motorSupplyCurrent, motorOutputVoltage);
        // Obtain Motor Readings
        inputs.motorConnected = statusCode.isOK();
        inputs.relativeEncoderAngle = relativeEncoderAngle.getValue();
        inputs.encoderVelocity = relativeEncoderVelocity.getValue();
        inputs.motorSupplyCurrent = motorSupplyCurrent.getValue();
        inputs.motorOutputVoltage = motorOutputVoltage.getValue();

        SmartDashboard.putNumber("Arm/Raw Encoder Reading", absoluteEncoder.get());
        SmartDashboard.putBoolean("Arm/Absolute Encoder Connected", absoluteEncoder.isConnected());
    }

    private VoltageOut voltageOut = new VoltageOut(Volts.zero());

    @Override
    public void setMotorOutput(Voltage voltage) {
        voltageOut.withOutput(voltage);
        armTalon.setControl(voltageOut);
    }

    @Override
    public void setMotorBrake(boolean brakeModeEnable) {
        NeutralModeValue value = brakeModeEnable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        armTalon.setNeutralMode(value);
    }
}
