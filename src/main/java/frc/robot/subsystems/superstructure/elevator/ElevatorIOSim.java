package frc.robot.subsystems.superstructure.elevator;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.superstructure.elevator.ElevatorConstants.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Robot;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

public class ElevatorIOSim implements ElevatorIO {
    private final ElevatorSim elevatorSim;
    private final SimulatedMotorController.GenericMotorController simMotorController;

    private Voltage requestedVoltage = Volts.zero();

    public ElevatorIOSim() {
        // Circumference = Drum Teeth Count * Chain Length
        double drumCircumferenceMeters = HARDWARE_CONSTANTS.ELEVATOR_DRUM_WHEEL_TEETH()
                * HARDWARE_CONSTANTS.CHAIN_LENGTH().in(Meters);
        // Radius = Circumference / pi / 2
        double drumRadiusMeters = drumCircumferenceMeters / Math.PI / 2;
        this.elevatorSim = new ElevatorSim(
                HARDWARE_CONSTANTS.ELEVATOR_GEARBOX(),
                HARDWARE_CONSTANTS.ELEVATOR_GEARING_REDUCTION() / HARDWARE_CONSTANTS.ELEVATOR_STAGES(),
                HARDWARE_CONSTANTS.ELEVATOR_CARRIAGE_WEIGHT().in(Kilograms),
                drumRadiusMeters,
                0,
                HARDWARE_CONSTANTS.ELEVATOR_MAX_HEIGHT().in(Meters),
                true,
                0);

        this.simMotorController =
                new SimulatedMotorController.GenericMotorController(HARDWARE_CONSTANTS.ELEVATOR_GEARBOX());
        simMotorController.withCurrentLimit(STATOR_CURRENT_LIMIT);
        SimulatedBattery.addElectricalAppliances(this::getSupplyCurrent);
        elevatorSim.update(0.0);
    }

    @Override
    public void updateInputs(ElevatorInputs inputs) {
        // Drum Rotations * Drum Teeth Count * Chain Length = Height
        // Drum Rotations = Height / Drum Teeth Count / Chain Length
        double drumAngleRotations = elevatorSim.getPositionMeters()
                / HARDWARE_CONSTANTS.ELEVATOR_STAGES()
                / HARDWARE_CONSTANTS.ELEVATOR_DRUM_WHEEL_TEETH()
                / HARDWARE_CONSTANTS.CHAIN_LENGTH().in(Meters);
        Angle motorAngle = Rotations.of(drumAngleRotations * HARDWARE_CONSTANTS.ELEVATOR_GEARING_REDUCTION());
        double drumVelocityRotationsPerSecond = elevatorSim.getVelocityMetersPerSecond()
                / HARDWARE_CONSTANTS.ELEVATOR_STAGES()
                / HARDWARE_CONSTANTS.ELEVATOR_DRUM_WHEEL_TEETH()
                / HARDWARE_CONSTANTS.CHAIN_LENGTH().in(Meters);
        AngularVelocity motorVelocity =
                RotationsPerSecond.of(drumVelocityRotationsPerSecond * HARDWARE_CONSTANTS.ELEVATOR_GEARING_REDUCTION());
        Voltage actualOutputVoltage =
                simMotorController.constrainOutputVoltage(motorAngle, motorVelocity, requestedVoltage);
        actualOutputVoltage = SimulatedBattery.clamp(actualOutputVoltage);
        if (DriverStation.isDisabled()) actualOutputVoltage = Volts.zero();
        elevatorSim.setInputVoltage(actualOutputVoltage.in(Volts));

        // Run 10 iterations of the physics simulation to improve accuracy
        for (int i = 0; i < 10; i++) elevatorSim.update(Robot.defaultPeriodSecs / 10.0);

        inputs.hardwareConnected = true;
        inputs.encoderAngleRad = motorAngle.in(Radians);
        inputs.encoderVelocityRadPerSec = motorVelocity.in(RadiansPerSecond);
        inputs.motorSupplyCurrentAmps = getSupplyCurrent().in(Amps);
        inputs.motorOutputVolts = actualOutputVoltage.in(Volts);
    }

    private Current getSupplyCurrent() {
        return Amps.of(elevatorSim.getCurrentDrawAmps());
    }

    @Override
    public void setMotorOutput(double volts) {
        this.requestedVoltage = Volts.of(volts);
    }
}
