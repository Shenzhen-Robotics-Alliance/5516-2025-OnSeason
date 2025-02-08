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
        double drumCircumferenceMeters = ELEVATOR_DRUM_WHEEL_TEETH * CHAN_LENGTH.in(Meters);
        // Radius = Circumference / pi / 2
        double drumRadiusMeters = drumCircumferenceMeters / Math.PI / 2;
        this.elevatorSim = new ElevatorSim(
                ELEVATOR_GEARBOX,
                ELEVATOR_GEARING_REDUCTION / ELEVATOR_STAGES,
                ELEVATOR_CARRIAGE_WEIGHT.in(Kilograms),
                drumRadiusMeters,
                0,
                ELEVATOR_MAX_HEIGHT.in(Meters),
                true,
                0);

        this.simMotorController = new SimulatedMotorController.GenericMotorController(ELEVATOR_GEARBOX);
        SimulatedBattery.addElectricalAppliances(this::getSupplyCurrent);
        elevatorSim.update(0.0);
    }

    @Override
    public void updateInputs(ElevatorInputs inputs) {
        // Drum Rotations * Drum Teeth Count * Chain Length = Height
        // Drum Rotations = Height / Drum Teeth Count / Chain Length
        double drumAngleRotations =
                elevatorSim.getPositionMeters() / ELEVATOR_STAGES / ELEVATOR_DRUM_WHEEL_TEETH / CHAN_LENGTH.in(Meters);
        Angle motorAngle = Rotations.of(drumAngleRotations * ELEVATOR_GEARING_REDUCTION);
        double drumVelocityRotationsPerSecond = elevatorSim.getVelocityMetersPerSecond()
                / ELEVATOR_STAGES
                / ELEVATOR_DRUM_WHEEL_TEETH
                / CHAN_LENGTH.in(Meters);
        AngularVelocity motorVelocity =
                RotationsPerSecond.of(drumVelocityRotationsPerSecond * ELEVATOR_GEARING_REDUCTION);
        Voltage actualOutputVoltage =
                simMotorController.constrainOutputVoltage(motorAngle, motorVelocity, requestedVoltage);
        actualOutputVoltage = SimulatedBattery.clamp(actualOutputVoltage);
        if (DriverStation.isDisabled()) actualOutputVoltage = Volts.zero();
        elevatorSim.setInputVoltage(actualOutputVoltage.in(Volts));
        // Run 10 iterations of the physics simulation to improve accuracy
        for (int i = 0; i < 10; i++) elevatorSim.update(Robot.defaultPeriodSecs / 10);

        inputs.hardwareConnected = true;
        inputs.encoderAngle = motorAngle;
        inputs.encoderVelocity = motorVelocity;
        inputs.motorSupplyCurrent = getSupplyCurrent();
        inputs.motorOutputVoltage = actualOutputVoltage;
    }

    private Current getSupplyCurrent() {
        return Amps.of(elevatorSim.getCurrentDrawAmps());
    }

    @Override
    public void setMotorOutput(Voltage voltage) {
        this.requestedVoltage = voltage;
    }
}
