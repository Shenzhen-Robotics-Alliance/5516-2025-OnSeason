package frc.robot.subsystems.coralholder;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utils.AlertsManager;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Volts;

public class CoralHolder extends SubsystemBase {
    // Hardware interface
    private final CoralHolderIO io;
    private final CoralHolderInputsAutoLogged inputs;

    // Triggers
    /** Whether the coral is anywhere inside the intake (one of any sensor triggered). */
    public final Trigger hasCoral;
    /** Whether the coral is in place (triggering both sensors). */
    public final Trigger coralInPlace;

    // Alerts
    private final Alert motorHardwareFaultsAlert;
    private final Alert sensor1HardwareFaultsAlert;
    private final Alert sensor2HardwareFaultsAlert;

    public CoralHolder(CoralHolderIO io) {
        this.io = io;
        inputs = new CoralHolderInputsAutoLogged();

        this.hasCoral = new Trigger(() -> inputs.firstSensorTriggered || inputs.secondSensorTriggered);
        this.coralInPlace = new Trigger(() -> inputs.firstSensorTriggered && inputs.secondSensorTriggered);

        this.motorHardwareFaultsAlert = AlertsManager.create("Coral Holder roller motor hardware faults detected!", Alert.AlertType.kError);
        this.sensor1HardwareFaultsAlert = AlertsManager.create("Coral Holder sensor 1 hardware faults detected!", Alert.AlertType.kError);
        this.sensor2HardwareFaultsAlert = AlertsManager.create("Coral Holder sensor 2 hardware faults detected!", Alert.AlertType.kError);
    }

    public boolean hardwareOK() {
        return inputs.firstSensorConnected && inputs.secondSensorConnected && inputs.motorConnected;
    }

    private void setVoltage(double volts) {
        if (hardwareOK())
            io.setRollerMotorOutput(Volts.of(volts));
        else
            io.setRollerMotorOutput(Volts.zero());
    }

    @Override
    public void periodic() {
        // Update inputs from IO and AdvantageKit.
        io.updateInputs(inputs);
        Logger.processInputs("CoralHolder", inputs);

        // Update alerts
        motorHardwareFaultsAlert.set(!inputs.motorConnected);
        sensor1HardwareFaultsAlert.set(!inputs.firstSensorConnected);
        sensor2HardwareFaultsAlert.set(!inputs.secondSensorConnected);
    }

    /**
     * <p>A sequence that intakes a Coral.</p>
     * <p>Will roll the coral in and move it to the holding position (where both sensor1 and sensor2 are triggered). </p>
     * */
    public Command intakeCoralSequence() {
        return Commands.sequence(
                // Run the rollers forward quickly until the coral hits the first sensor
                run(() -> setVoltage(3.5)).until(hasCoral),
                // Run the rollers backwards for 0.1 for a rapid brake
                run(() -> setVoltage(-1)).withTimeout(0.1),
                // Next, run the rollers forward slowly until the coal hits the second sensor
                run(() -> setVoltage(0.8)).until(coralInPlace))
                // Only run when the rollers are not in place yet
                .onlyIf(coralInPlace.negate())
                // Stop the intake at the end of the command
                .finallyDo(() -> setVoltage(0.0));
    }

    /**
     * <p>Shuffles the coral such that.</p>
     * <p>This is used to move the coral to the appropriate position for scoring.</p>
     * */
    public Command shuffleCoralSequence() {
        return Commands.sequence(
                // If the coral is not in place (triggering sensor 2) yet,
                // we run rollers slowly forward until it triggers sensor 2.
                run(() -> setVoltage(1.2)).onlyIf(coralInPlace.negate()).until(coralInPlace),
                // Next, run the rollers slowly backwards until it does not trigger sensor 2
                run(() -> setVoltage(-0.8)).until(coralInPlace.negate()))
                // Only shuffle the coral if we have a coral.
                .onlyIf(hasCoral)
                // Stop the intake at the end of the command.
                .finallyDo(() -> setVoltage(0.0));
    }

    /**
     * <p>Score the Coral inside the holder.</p>
     * */
    public Command scoreCoral() {
        return run(() -> setVoltage(3.5))
                .until(hasCoral.negate())
                .finallyDo(() -> setVoltage(0.0));
    }
}
