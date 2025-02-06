package frc.robot.subsystems.superstructure.arm;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.superstructure.arm.ArmConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import java.util.Optional;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

public class ArmIOSim implements ArmIO {
    private final SingleJointedArmSim armSim;
    private final SimulatedMotorController.GenericMotorController simMotorController;

    private final Angle relativeEncoderOffset;
    private Voltage requestedVoltage;

    public ArmIOSim() {
        this.armSim = new SingleJointedArmSim(
                ARM_GEARBOX,
                ARM_GEARING_REDUCTION,
                SingleJointedArmSim.estimateMOI(ARM_LENGTH.in(Meters), ARM_MASS.in(Kilograms)),
                ARM_LENGTH.in(Meters),
                ARM_LOWER_LIMIT.in(Radians),
                ARM_UPPER_LIMIT.in(Radians),
                true,
                ARM_UPPER_LIMIT.in(Radians));

        this.simMotorController =
                new SimulatedMotorController.GenericMotorController(ARM_GEARBOX).withCurrentLimit(ARM_CURRENT_LIMIT);
        this.relativeEncoderOffset = Rotations.of((Math.random() - 0.5) * 5);
    }

    @Override
    public void updateInputs(ArmInputs armInputs) {
        Voltage actualOutputVoltage = simMotorController.constrainOutputVoltage(
                Radians.of(armSim.getAngleRads() * ARM_GEARING_REDUCTION),
                RadiansPerSecond.of(armSim.getVelocityRadPerSec() * ARM_GEARING_REDUCTION),
                requestedVoltage);
        armSim.setInputVoltage(actualOutputVoltage.in(Volts));

        armInputs.absoluteEncoderAngle = Optional.of(Rotation2d.fromRadians(armSim.getAngleRads()));
        armInputs.motorConnected = true;
        armInputs.relativeMechanismAngle = Radians.of(armSim.getAngleRads()).plus(relativeEncoderOffset);
        armInputs.mechanismVelocity = RadiansPerSecond.of(armSim.getVelocityRadPerSec());
        armInputs.motorSupplyCurrent = Amps.of(armSim.getCurrentDrawAmps());
        armInputs.motorOutputVoltage = actualOutputVoltage;
    }

    @Override
    public void setMotorOutput(Voltage voltage) {
        this.requestedVoltage = voltage;
    }
}
