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
    private final TalonFX elevatorTalon1;
    private final TalonFX elevatorTalon2;

    // CTRE Status Signals
    private final StatusSignal<Angle> motor1Position;
    private final StatusSignal<AngularVelocity> motor1Velocity;
    private final StatusSignal<Current> motor1SupplyCurrent;
    private final StatusSignal<Current> motor2SupplyCurrent;
    private final StatusSignal<Voltage> motor1OutputVoltage;
    private final StatusSignal<Voltage> motor2OutputVoltage;

    public ElevatorIOReal() {
        this.elevatorTalon1 = new TalonFX(1);
        this.elevatorTalon2 = new TalonFX(2);
        CurrentLimitsConfigs currentLimitsConfigs =
                new CurrentLimitsConfigs().withSupplyCurrentLimitEnable(true).withSupplyCurrentLimit(CURRENT_LIMIT);
        this.elevatorTalon1.getConfigurator().apply(currentLimitsConfigs);
        this.elevatorTalon1
                .getConfigurator()
                .apply(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));
        this.elevatorTalon2.getConfigurator().apply(currentLimitsConfigs);
        this.elevatorTalon2
                .getConfigurator()
                .apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

        this.motor1Position = elevatorTalon1.getPosition();
        this.motor1Velocity = elevatorTalon1.getVelocity();
        this.motor1SupplyCurrent = elevatorTalon1.getSupplyCurrent();
        this.motor2SupplyCurrent = elevatorTalon2.getSupplyCurrent();
        this.motor1OutputVoltage = elevatorTalon1.getMotorVoltage();
        this.motor2OutputVoltage = elevatorTalon2.getMotorVoltage();

        BaseStatusSignal.setUpdateFrequencyForAll(
                100.0,
                motor1Position,
                motor1Velocity,
                motor1SupplyCurrent,
                motor2SupplyCurrent,
                motor1OutputVoltage,
                motor2OutputVoltage);

        elevatorTalon1.optimizeBusUtilization();
        elevatorTalon2.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ElevatorInputs inputs) {
        StatusCode statusCode = BaseStatusSignal.refreshAll(
                motor1Position,
                motor1Velocity,
                motor1SupplyCurrent,
                motor2SupplyCurrent,
                motor1OutputVoltage,
                motor2OutputVoltage);

        inputs.hardwareConnected = statusCode.isOK();
        inputs.encoderAngle = motor1Position.getValue();
        inputs.encoderVelocity = motor1Velocity.getValue();
        inputs.motorSupplyCurrent = motor1SupplyCurrent.getValue().plus(motor2SupplyCurrent.getValue());
        inputs.motorOutputVoltage = motor1OutputVoltage
                .getValue()
                .plus(motor2OutputVoltage.getValue())
                .div(2);
    }

    private VoltageOut voltageOut = new VoltageOut(Volts.zero());

    @Override
    public void setMotorOutput(Voltage voltage) {
        voltageOut.withOutput(voltage);
        elevatorTalon1.setControl(voltageOut);
        elevatorTalon2.setControl(voltageOut);
    }

    @Override
    public void setMotorBrake(boolean brakeModeEnable) {
        NeutralModeValue value = brakeModeEnable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    }
}
