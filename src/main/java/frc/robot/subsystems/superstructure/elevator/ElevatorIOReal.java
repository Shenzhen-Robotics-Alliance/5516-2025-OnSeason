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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class ElevatorIOReal implements ElevatorIO {
    // Hardware
    private final TalonFX elevatorTalon;

    // CTRE Status Signals
    private final StatusSignal<Angle> motor1Position;
    private final StatusSignal<AngularVelocity> motor1Velocity;
    private final StatusSignal<Current> motorSupplyCurrent;
    private final StatusSignal<Voltage> motorOutputVoltage;

    public ElevatorIOReal() {
        this.elevatorTalon = new TalonFX(1);
        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(STATOR_CURRENT_LIMIT)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLowerTime(OVERHEAT_PROTECTION_TIME)
                .withSupplyCurrentLowerLimit(OVERHEAT_PROTECTION_CURRENT);
        this.elevatorTalon.getConfigurator().apply(currentLimitsConfigs);
        this.elevatorTalon
                .getConfigurator()
                .apply(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));

        this.motor1Position = elevatorTalon.getPosition();
        this.motor1Velocity = elevatorTalon.getVelocity();
        this.motorSupplyCurrent = elevatorTalon.getSupplyCurrent();
        this.motorOutputVoltage = elevatorTalon.getMotorVoltage();

        BaseStatusSignal.setUpdateFrequencyForAll(
                100.0, motor1Position, motor1Velocity, motorSupplyCurrent, motorOutputVoltage);

        elevatorTalon.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ElevatorInputs inputs) {
        StatusCode statusCode =
                BaseStatusSignal.refreshAll(motor1Position, motor1Velocity, motorSupplyCurrent, motorOutputVoltage);

        inputs.hardwareConnected = statusCode.isOK();
        inputs.encoderAngle = motor1Position.getValue();
        inputs.encoderVelocity = motor1Velocity.getValue();
        inputs.motorSupplyCurrent = motorSupplyCurrent.getValue();
        inputs.motorOutputVoltage = motorOutputVoltage.getValue();
    }

    private VoltageOut voltageOut = new VoltageOut(Volts.zero());

    @Override
    public void setMotorOutput(Voltage voltage) {
        voltageOut.withOutput(voltage);
        elevatorTalon.setControl(voltageOut);
    }

    @Override
    public void setMotorBrake(boolean brakeModeEnable) {
        NeutralModeValue value = brakeModeEnable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        elevatorTalon.setNeutralMode(value);
    }
}
