package frc.robot.subsystems.coralholder;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.coralholder.CoralHolderConstants.*;

import au.grapplerobotics.CanBridge;
import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface;
import com.ctre.phoenix6.BaseStatusSignal;
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
import java.util.Arrays;

public class CoralHolderIOReal implements CoralHolderIO {
    private final TalonFX rollerTalon;
    private final TalonFX[] feederTalons;
    private final LaserCan firstSensor, secondSensor;

    private final StatusSignal<Current> rollerMotorCurrent;
    private final StatusSignal<Voltage> rollerMotorOutputVoltage;

    private final StatusSignal<Current>[] feedersCurrent;
    private final StatusSignal<Voltage>[] feedersOutput;

    private final boolean firstSensorConfigurationError;
    private final boolean secondSensorConfigurationError;

    public CoralHolderIOReal() {
        this.rollerTalon = new TalonFX(HARDWARE_CONSTANTS.rollerMotorID());
        CurrentLimitsConfigs rollerCurrentLimit = new CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(ROLLERS_CURRENT_LIMIT);
        this.rollerTalon.getConfigurator().apply(rollerCurrentLimit);
        rollerTalon
                .getConfigurator()
                .apply(new MotorOutputConfigs()
                        .withInverted(
                                HARDWARE_CONSTANTS.rollerMotorInverted()
                                        ? InvertedValue.Clockwise_Positive
                                        : InvertedValue.CounterClockwise_Positive));
        this.rollerTalon.setNeutralMode(NeutralModeValue.Brake);

        this.rollerMotorCurrent = rollerTalon.getSupplyCurrent();
        this.rollerMotorOutputVoltage = rollerTalon.getMotorVoltage();

        this.feederTalons = Arrays.stream(HARDWARE_CONSTANTS.feederMotorsIDs())
                .mapToObj(TalonFX::new)
                .toArray(TalonFX[]::new);
        this.feedersCurrent =
                Arrays.stream(feederTalons).map(TalonFX::getSupplyCurrent).toArray(StatusSignal[]::new);
        this.feedersOutput =
                Arrays.stream(feederTalons).map(TalonFX::getMotorVoltage).toArray(StatusSignal[]::new);

        double freq = 100.0;
        BaseStatusSignal.setUpdateFrequencyForAll(freq, rollerMotorCurrent, rollerMotorOutputVoltage);
        BaseStatusSignal.setUpdateFrequencyForAll(freq, feedersCurrent);
        BaseStatusSignal.setUpdateFrequencyForAll(freq, feedersOutput);
        this.rollerTalon.optimizeBusUtilization();
        for (int i = 0; i < feederTalons.length; i++) {
            feederTalons[i]
                    .getConfigurator()
                    .apply(new MotorOutputConfigs()
                            .withInverted(
                                    HARDWARE_CONSTANTS.feederMotorsInverted()[i]
                                            ? InvertedValue.Clockwise_Positive
                                            : InvertedValue.CounterClockwise_Positive));
            feederTalons[i]
                    .getConfigurator()
                    .apply(new CurrentLimitsConfigs()
                            .withSupplyCurrentLimitEnable(true)
                            .withSupplyCurrentLimit(40));
            feederTalons[i].optimizeBusUtilization();
        }

        this.firstSensor = new LaserCan(HARDWARE_CONSTANTS.firstSensorID());
        this.secondSensor = new LaserCan(HARDWARE_CONSTANTS.secondSensorID());
        this.firstSensorConfigurationError = !configureSensor(firstSensor);
        this.secondSensorConfigurationError = !configureSensor(secondSensor);

        CanBridge.runTCP();
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
            sensor.setRegionOfInterest(new LaserCanInterface.RegionOfInterest(8, 8, 6, 6));
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
        inputs.motorConnected = BaseStatusSignal.refreshAll(rollerMotorCurrent, rollerMotorOutputVoltage)
                .isOK();
        if (feederTalons.length > 0) {
            inputs.motorConnected = inputs.motorConnected
                    && BaseStatusSignal.refreshAll(feedersOutput).isOK();
            inputs.motorConnected = inputs.motorConnected
                    && BaseStatusSignal.refreshAll(feedersCurrent).isOK();
        }
        inputs.rollerMotorCurrentAmps = rollerMotorCurrent.getValueAsDouble();
        inputs.rollerMotorOutputVolts = rollerMotorOutputVoltage.getValueAsDouble();
        inputs.feederMotorCurrentAmps = inputs.feederMotorOutputVolts = 0;
        for (int i = 0; i < feederTalons.length; i++) {
            inputs.feederMotorOutputVolts += feedersCurrent[i].getValueAsDouble();
            inputs.feederMotorCurrentAmps += feedersOutput[i].getValueAsDouble();
        }
        inputs.feederMotorOutputVolts /= feederTalons.length;

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
    public void setRollerMotorOutput(double volts) {
        voltageOut.withOutput(volts);
        rollerTalon.setControl(voltageOut);
    }

    @Override
    public void setCollectorMotorOutput(double volts) {
        voltageOut.withOutput(volts);
        for (TalonFX feederTalon : feederTalons) feederTalon.setControl(voltageOut);
    }
}
