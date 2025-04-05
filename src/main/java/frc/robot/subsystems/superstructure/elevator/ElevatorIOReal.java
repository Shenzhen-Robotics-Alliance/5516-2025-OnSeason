package frc.robot.subsystems.superstructure.elevator;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.superstructure.elevator.ElevatorConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import java.util.Optional;

public class ElevatorIOReal implements ElevatorIO {
    // Hardware
    private final TalonFX masterTalon;
    private final Optional<TalonFX> followerTalon;

    // CTRE Status Signals
    private final StatusSignal<Angle> masterPosition;
    private final StatusSignal<AngularVelocity> masterVelocity;
    private final StatusSignal<Current> masterSupplyCurrent;
    private final StatusSignal<Voltage> masterOutputVoltage;
    private final StatusSignal<Temperature> masterTemperature;

    public ElevatorIOReal() {
        this.masterTalon = new TalonFX(HARDWARE_CONSTANTS.ELEVATOR_MOTOR_ID());
        this.followerTalon = HARDWARE_CONSTANTS.ELEVATOR_FOLLOWER_ID().stream()
                .mapToObj(TalonFX::new)
                .findAny();
        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(STATOR_CURRENT_LIMIT)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLowerTime(OVERHEAT_PROTECTION_TIME)
                .withSupplyCurrentLowerLimit(OVERHEAT_PROTECTION_CURRENT);
        this.masterTalon.getConfigurator().apply(currentLimitsConfigs);
        this.masterTalon
                .getConfigurator()
                .apply(new MotorOutputConfigs()
                        .withInverted(
                                HARDWARE_CONSTANTS.ELEVATOR_MOTOR_INVERTED()
                                        ? InvertedValue.Clockwise_Positive
                                        : InvertedValue.CounterClockwise_Positive));

        this.followerTalon.ifPresent((talon) -> {
            talon.getConfigurator().apply(currentLimitsConfigs);
            talon.getConfigurator()
                    .apply(new MotorOutputConfigs()
                            .withInverted(
                                    HARDWARE_CONSTANTS.ELEVATOR_FOLLOWER_INVERTED()
                                            ? InvertedValue.Clockwise_Positive
                                            : InvertedValue.CounterClockwise_Positive));
        });

        this.masterPosition = masterTalon.getPosition();
        this.masterVelocity = masterTalon.getVelocity();

        this.masterSupplyCurrent = masterTalon.getSupplyCurrent();
        this.masterOutputVoltage = masterTalon.getMotorVoltage();
        this.masterTemperature = masterTalon.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
                100.0, masterPosition, masterVelocity, masterSupplyCurrent, masterOutputVoltage, masterTemperature);

        masterTalon.optimizeBusUtilization();
        followerTalon.ifPresent(TalonFX::optimizeBusUtilization);
        masterTalon.setPosition(0.0);
    }

    @Override
    public void updateInputs(ElevatorInputs inputs) {
        StatusCode statusCode = BaseStatusSignal.refreshAll(
                masterPosition, masterVelocity, masterSupplyCurrent, masterOutputVoltage, masterTemperature);

        inputs.hardwareConnected = statusCode.isOK();
        inputs.followerConnected =
                followerTalon.isEmpty() || followerTalon.get().isAlive();
        inputs.encoderAngleRad = Units.rotationsToRadians(masterPosition.getValueAsDouble());
        inputs.encoderVelocityRadPerSec = Units.rotationsToRadians(masterVelocity.getValueAsDouble());
        inputs.motorSupplyCurrentAmps = masterSupplyCurrent.getValueAsDouble();
        inputs.motorOutputVolts = masterOutputVoltage.getValueAsDouble();
        inputs.motorTemperatureCelsius = masterTemperature.getValueAsDouble();
    }

    private VoltageOut voltageOut = new VoltageOut(Volts.zero());

    @Override
    public void setMotorOutput(double volts) {
        voltageOut.withOutput(volts);
        masterTalon.setControl(voltageOut);
        followerTalon.ifPresent(talonFX -> talonFX.setControl(voltageOut));
    }

    @Override
    public void setMotorBrake(boolean brakeModeEnable) {
        NeutralModeValue value = brakeModeEnable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        masterTalon.setNeutralMode(value);
        followerTalon.ifPresent(talonFX -> talonFX.setNeutralMode(value));
    }

    @Override
    public void zeroEncoder() {
        masterTalon.setPosition(0.0);
    }
}
