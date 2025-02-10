package frc.robot.subsystems.coralholder;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.coralholder.CoralHolderConstants.*;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;

public class CoralHolderIOReal implements CoralHolderIO {
    private final TalonFX rollerTalon, feederTalon1, feederTalon2;
    private final LaserCan firstSensor, secondSensor;

    private final StatusSignal<Current> rollerMotorCurrent;
    private final StatusSignal<Voltage> rollerMotorOutputVoltage;

    private final StatusSignal<Current> feeder1Current, feeder2Current;
    private final StatusSignal<Voltage> feeder1Output, feeder2Output;

    private final boolean firstSensorConfigurationError;
    private final boolean secondSensorConfigurationError;

    public CoralHolderIOReal() {
        this.rollerTalon = new TalonFX(3);
        CurrentLimitsConfigs rollerCurrentLimit = new CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(ROLLERS_CURRENT_LIMIT);
        this.rollerTalon.getConfigurator().apply(rollerCurrentLimit);
        rollerTalon.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
        this.rollerTalon.setNeutralMode(NeutralModeValue.Brake);

        this.rollerMotorCurrent = rollerTalon.getSupplyCurrent();
        this.rollerMotorOutputVoltage = rollerTalon.getMotorVoltage();

        this.feederTalon1 = new TalonFX(5);
        this.feederTalon2 = new TalonFX(6);
        CurrentLimitsConfigs feederCurrentLimit =
                new CurrentLimitsConfigs().withSupplyCurrentLimitEnable(true).withSupplyCurrentLimit(40);
        feederTalon1.getConfigurator().apply(feederCurrentLimit);
        feederTalon1.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
        feederTalon2.getConfigurator().apply(feederCurrentLimit);
        feederTalon2
                .getConfigurator()
                .apply(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));

        feeder1Current = feederTalon1.getSupplyCurrent();
        feeder2Current = feederTalon2.getStatorCurrent();
        feeder1Output = feederTalon1.getMotorVoltage();
        feeder2Output = feederTalon2.getMotorVoltage();

        BaseStatusSignal.setUpdateFrequencyForAll(
                100.0,
                rollerMotorCurrent,
                rollerMotorOutputVoltage,
                feeder1Current,
                feeder2Current,
                feeder1Output,
                feeder2Output);
        this.rollerTalon.optimizeBusUtilization();

        this.firstSensor = new LaserCan(0);
        this.secondSensor = new LaserCan(1);
        this.firstSensorConfigurationError = !configureSensor(firstSensor);
        this.secondSensorConfigurationError = !configureSensor(secondSensor);
    }

    /**
     * Attempts to configure the laser can sensor.
     *
     * @return whether the attempt is successful
     */
    private static boolean configureSensor(LaserCan sensor) {
        try {
            sensor.setRangingMode(LaserCanInterface.RangingMode.SHORT);
            sensor.setTimingBudget(LaserCanInterface.TimingBudget.TIMING_BUDGET_20MS);
            return true;
        } catch (ConfigurationFailedException e) {
            return false;
        }
    }

    private static boolean isTriggered(LaserCanInterface.Measurement measurement, Distance threshold) {
        if (measurement.status != LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) return false;
        return measurement.distance_mm < threshold.in(Millimeters);
    }

    @Override
    public void updateInputs(CoralHolderInputs inputs) {
        StatusCode statusCode = BaseStatusSignal.refreshAll(
                rollerMotorCurrent,
                rollerMotorOutputVoltage,
                feeder1Current,
                feeder2Current,
                feeder1Output,
                feeder2Output);
        inputs.motorConnected = statusCode.isOK();
        inputs.rollerMotorCurrent = rollerMotorCurrent.getValue();
        inputs.rollerMotorOutputVoltage = rollerMotorOutputVoltage.getValue();
        inputs.feederMotorCurrent = feeder1Current.getValue().plus(feeder2Current.getValue());
        inputs.feederMotorOutputVoltage =
                feeder1Output.getValue().plus(feeder2Output.getValue()).div(2);

        LaserCanInterface.Measurement firstSensorMeasurement = firstSensor.getMeasurement();
        if (firstSensorMeasurement == null || firstSensorConfigurationError) {
            inputs.firstSensorDistanceMM = Double.POSITIVE_INFINITY;
            inputs.firstSensorConnected = inputs.firstSensorReadingValid = false;
        } else {
            inputs.firstSensorConnected = true;
            inputs.firstSensorReadingValid =
                    firstSensorMeasurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT;
            inputs.firstSensorDistanceMM = firstSensorMeasurement.distance_mm;
        }

        LaserCanInterface.Measurement secondSensorMeasurement = secondSensor.getMeasurement();
        if (secondSensorMeasurement == null || secondSensorConfigurationError) {
            inputs.secondSensorConnected = inputs.secondSensorReadingValid = false;
        } else {
            inputs.secondSensorConnected = true;
            inputs.secondSensorReadingValid =
                    secondSensorMeasurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT;
            inputs.secondSensorDistanceMM = secondSensorMeasurement.distance_mm;
        }
    }

    VoltageOut voltageOut = new VoltageOut(Volts.zero());

    @Override
    public void setRollerMotorOutput(Voltage voltage) {
        voltageOut.withOutput(voltage);
        rollerTalon.setControl(voltageOut);
    }

    @Override
    public void setCollectorMotorOutput(Voltage voltage) {
        voltageOut.withOutput(voltage);
        feederTalon1.setControl(voltageOut);
        feederTalon2.setControl(voltageOut);
    }
}
