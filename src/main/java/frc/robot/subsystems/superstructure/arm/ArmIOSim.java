package frc.robot.subsystems.superstructure.arm;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.superstructure.arm.ArmConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Robot;
import java.util.Optional;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

public class ArmIOSim implements ArmIO {
    private final SingleJointedArmSim armSim;
    private final SimulatedMotorController.GenericMotorController simMotorController;

    private final Angle relativeEncoderOffset;
    private Voltage requestedVoltage;

    public ArmIOSim() {
        this.armSim = new SingleJointedArmSim(
                HARDWARE_CONSTANTS.ARM_GEARBOX(),
                HARDWARE_CONSTANTS.ARM_GEARING_REDUCTION(),
                SingleJointedArmSim.estimateMOI(
                        HARDWARE_CONSTANTS.ARM_COM_LENGTH().in(Meters),
                        HARDWARE_CONSTANTS.ARM_MASS().in(Kilograms)),
                HARDWARE_CONSTANTS.ARM_COM_LENGTH().in(Meters),
                HARDWARE_CONSTANTS.ARM_LOWER_HARD_LIMIT().in(Radians),
                HARDWARE_CONSTANTS.ARM_UPPER_HARD_LIMIT().in(Radians),
                true,
                HARDWARE_CONSTANTS.ARM_UPPER_HARD_LIMIT().in(Radians));

        this.simMotorController = new SimulatedMotorController.GenericMotorController(HARDWARE_CONSTANTS.ARM_GEARBOX())
                .withCurrentLimit(ARM_CURRENT_LIMIT);
        this.relativeEncoderOffset = Rotations.of((Math.random() - 0.5) * 10);
        this.requestedVoltage = Volts.zero();

        SimulatedBattery.addElectricalAppliances(this::getSupplyCurrent);
        armSim.update(0.0);
    }

    @Override
    public void updateInputs(ArmInputs armInputs) {
        Angle motorAngle = Radians.of(armSim.getAngleRads() * HARDWARE_CONSTANTS.ARM_GEARING_REDUCTION());
        AngularVelocity motorAngularVelocity =
                RadiansPerSecond.of(armSim.getVelocityRadPerSec() * HARDWARE_CONSTANTS.ARM_GEARING_REDUCTION());
        Voltage actualOutputVoltage =
                simMotorController.constrainOutputVoltage(motorAngle, motorAngularVelocity, requestedVoltage);
        actualOutputVoltage = SimulatedBattery.clamp(actualOutputVoltage);
        if (DriverStation.isDisabled()) actualOutputVoltage = Volts.zero();
        armSim.setInputVoltage(actualOutputVoltage.in(Volts));

        // Run 10 iterations of the physics simulation to improve accuracy
        for (int i = 0; i < 10; i++) armSim.update(Robot.defaultPeriodSecs / 10);

        armInputs.absoluteEncoderAngle = Optional.of(Rotation2d.fromRadians(armSim.getAngleRads()));
        armInputs.motorConnected = true;
        armInputs.relativeEncoderAngleRad =
                motorAngle.plus(relativeEncoderOffset).in(Radians);
        armInputs.encoderVelocityRadPerSec = motorAngularVelocity.in(RadiansPerSecond);
        armInputs.motorSupplyCurrentAmps = Amps.of(armSim.getCurrentDrawAmps()).in(Amps);
        armInputs.motorOutputVolts = actualOutputVoltage.in(Volts);
    }

    private Current getSupplyCurrent() {
        return Amps.of(armSim.getCurrentDrawAmps());
    }

    @Override
    public void setMotorOutput(Voltage voltage) {
        this.requestedVoltage = voltage;
    }
}
