package frc.robot.subsystems.coralholder;

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
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.coralholder.CoralHolderConstants.*;

public class CoralHolderIOReal implements CoralHolderIO {
    private final TalonFX rollerTalon;
    private final LaserCan firstSensor;
    private final LaserCan secondSensor;

    private final StatusSignal<Current> rollerMotorCurrent;
    private final StatusSignal<Voltage> rollerMotorOutputVoltage;

    private final boolean firstSensorConfigurationError;
    private final boolean secondSensorConfigurationError;

    public CoralHolderIOReal() {
        this.rollerTalon = new TalonFX(3);
        this.rollerTalon.getConfigurator().apply(new CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(ROLLERS_CURRENT_LIMIT));
        rollerTalon.getConfigurator().apply(new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive));
        this.rollerTalon.setNeutralMode(NeutralModeValue.Brake);

        this.rollerMotorCurrent = rollerTalon.getSupplyCurrent();
        this.rollerMotorOutputVoltage = rollerTalon.getMotorVoltage();
        BaseStatusSignal.setUpdateFrequencyForAll(100.0, rollerMotorCurrent, rollerMotorOutputVoltage);
        this.rollerTalon.optimizeBusUtilization();

        this.firstSensor = new LaserCan(0);
        this.secondSensor = new LaserCan(1);
        this.firstSensorConfigurationError = !configureSensor(firstSensor);
        this.secondSensorConfigurationError = !configureSensor(secondSensor);
    }

    /**
     * <p>Attempts to configure the laser can sensor.</p>
     * @return whether the attempt is successful
     * */
    private static boolean configureSensor(LaserCan sensor) {
        try {
            sensor.setRangingMode(LaserCanInterface.RangingMode.SHORT);
            sensor.setTimingBudget(LaserCanInterface.TimingBudget.TIMING_BUDGET_20MS);
            return true;
        } catch (ConfigurationFailedException e) {
            return false;
        }
    }

    private static boolean isTriggered(LaserCanInterface.Measurement measurement) {
        if (measurement.status != LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT)
            return false;
        return measurement.distance_mm < SENSOR_DISTANCE_THRESHOLD.in(Millimeters);
    }

    @Override
    public void updateInputs(CoralHolderInputs inputs) {
        StatusCode statusCode = BaseStatusSignal.refreshAll(rollerMotorCurrent, rollerMotorOutputVoltage);
        inputs.motorConnected = statusCode.isOK();
        inputs.rollerMotorCurrent = rollerMotorCurrent.getValue();
        inputs.rollerMotorOutputVoltage = rollerMotorOutputVoltage.getValue();

        LaserCanInterface.Measurement firstSensorMeasurement = firstSensor.getMeasurement();
        if (firstSensorMeasurement == null || firstSensorConfigurationError)
            inputs.firstSensorConnected = inputs.firstSensorTriggered = false;
        else {
            inputs.firstSensorConnected = true;
            inputs.firstSensorTriggered = isTriggered(firstSensorMeasurement);
        }

        LaserCanInterface.Measurement secondSensorMeasurement = secondSensor.getMeasurement();
        if (secondSensorMeasurement == null || secondSensorConfigurationError)
            inputs.secondSensorConnected = inputs.secondSensorTriggered = false;
        else {
            inputs.secondSensorConnected = true;
            inputs.secondSensorTriggered = isTriggered(secondSensorMeasurement);
        }
    }

    VoltageOut voltageOut = new VoltageOut(Volts.zero());
    @Override
    public void setRollerMotorOutput(Voltage voltage) {
        voltageOut.withOutput(voltage);
        rollerTalon.setControl(voltageOut);
    }
}
