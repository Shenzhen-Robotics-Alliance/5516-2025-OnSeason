package frc.robot.subsystems.superstructure.elevator;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.superstructure.elevator.ElevatorConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.utils.AlertsManager;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
    // Hardware interface
    private final ElevatorIO io;
    private final ElevatorInputsAutoLogged inputs;

    // Controllers
    private final ElevatorFeedforward feedforwardController;
    private final PIDController strongFeedbackController;
    private final PIDController wearkFeedbackController;
    private final TrapezoidProfile profile;

    // Alerts
    private final Alert elevatorHardwareFaultsAlert;
    private final Alert elevatorExceedLimitAlert;

    /** Debounce for hardware faults */
    private final Debouncer hardwareFaultDebouncer;

    /** Whether there is a hardware fault in the motor. */
    private boolean hardwareFaultDetected;

    /** The current state. */
    private TrapezoidProfile.State currentStateMeters;

    /** The desired height. */
    private Optional<Distance> heightSetpoint;

    public Elevator(ElevatorIO io) {
        this.io = io;
        this.inputs = new ElevatorInputsAutoLogged();

        this.feedforwardController = new ElevatorFeedforward(kS, kG, kV, kA);
        this.strongFeedbackController = new PIDController(kP_STRONG, 0, 0);
        this.wearkFeedbackController = new PIDController(kP_WEAK, 0, 0);
        this.profile = new TrapezoidProfile(PROFILE_CONSTRAINS);

        this.elevatorExceedLimitAlert = AlertsManager.create("", Alert.AlertType.kError);
        this.elevatorExceedLimitAlert.set(false);
        this.elevatorHardwareFaultsAlert =
                AlertsManager.create("Elevator hardware faults detected!", Alert.AlertType.kError);
        this.hardwareFaultDebouncer = new Debouncer(0.3, Debouncer.DebounceType.kRising);
        this.elevatorHardwareFaultsAlert.set(false);

        this.currentStateMeters = new TrapezoidProfile.State(0, 0);
        this.heightSetpoint = Optional.empty();

        io.setMotorBrake(true);
    }

    /**
     * Sets the motors to idle.
     *
     * <p>The elevator will slide down due to gravity.
     *
     * <p><b>Caution: The elevator will hit the basement really hard if brake mode is off.</b>
     */
    private void executeIdle() {
        io.setMotorOutput(Volts.zero());
        currentStateMeters = new TrapezoidProfile.State(getHeight().in(Meters), 0);
        previousVelocityMPS = getVelocity().in(MetersPerSecond);
    }

    private double previousVelocityMPS = 0.0;

    /** Runs the control loops on the elevator to achieve the setpoint. */
    private void executeControlLoops(Distance heightSetpoint) {
        TrapezoidProfile.State goalState = new TrapezoidProfile.State(heightSetpoint.in(Meters), 0);
        currentStateMeters = profile.calculate(Robot.defaultPeriodSecs, currentStateMeters, goalState);

        double accelerationMPSSq = (currentStateMeters.velocity - previousVelocityMPS) / Robot.defaultPeriodSecs;
        double feedforwardVolts = feedforwardController.calculate(currentStateMeters.velocity, accelerationMPSSq);
        boolean elevatorRequestedToMove =
                Math.abs(currentStateMeters.velocity) > ELEVATOR_MOVING_VELOCITY_THRESHOLD.in(MetersPerSecond);
        PIDController feedbackController = elevatorRequestedToMove ? strongFeedbackController : wearkFeedbackController;
        double feedbackVolts = feedbackController.calculate(getHeight().in(Meters), currentStateMeters.position);

        Voltage voltage = Volts.of(MathUtil.clamp(
                feedforwardVolts + feedbackVolts, MIN_OUTPUT_VOLTAGE.in(Volts), MAX_OUTPUT_VOLTAGE.in(Volts)));
        io.setMotorOutput(voltage);
    }

    /** @return the measured elevator height, where zero is lowest. */
    public Distance getHeight() {
        // Height = Drum Rotations * Drum Teeth Count * Chain Length
        return CHAN_LENGTH.times(
                inputs.encoderAngle.in(Rotations) / ELEVATOR_GEARING_REDUCTION * ELEVATOR_DRUM_WHEEL_TEETH);
    }

    /** @return the measured elevator velocity, where positive is up. */
    public LinearVelocity getVelocity() {
        return CHAN_LENGTH
                .times(inputs.encoderVelocity.in(RotationsPerSecond)
                        / ELEVATOR_GEARING_REDUCTION
                        * ELEVATOR_DRUM_WHEEL_TEETH)
                .per(Seconds);
    }

    /**
     * Whether the elevator is close enough to its setpoint.
     *
     * @return <code>true</code> if there is a setpoint and the elevator is close enough to it, <code>false</code>
     *     otherwise.
     */
    public boolean atReference() {
        if (heightSetpoint.isEmpty()) return false;
        return getHeight().minus(heightSetpoint.get()).abs(Meters) < ELEVATOR_PID_TOLERANCE.in(Meters);
    }

    @Override
    public void periodic() {
        // Update inputs from IO and AdvantageKit
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);

        // Disable PID setpoint if the robot is disabled.
        if (DriverStation.isDisabled()) heightSetpoint = Optional.empty();
        // Run setpoints (or run idle if no setpoint).
        heightSetpoint.ifPresentOrElse(this::executeControlLoops, this::executeIdle);

        // Update Alerts
        hardwareFaultDetected = hardwareFaultDebouncer.calculate(!inputs.hardwareConnected);
        elevatorHardwareFaultsAlert.set(hardwareFaultDetected);
        if (getHeight().minus(ELEVATOR_PID_TOLERANCE).lt(Meters.zero())) {
            elevatorExceedLimitAlert.setText(
                    "Elevator height exceeds lower limit: " + getHeight().in(Meters) + " Meters");
            elevatorExceedLimitAlert.set(true);
        } else if (getHeight().plus(ELEVATOR_PID_TOLERANCE).gt(ELEVATOR_MAX_HEIGHT)) {
            elevatorExceedLimitAlert.setText(
                    "Elevator height exceeds higher limit: " + getHeight().in(Meters) + " Meters");
            elevatorExceedLimitAlert.set(true);
        } else elevatorExceedLimitAlert.set(false);
    }

    /**
     * Moves to a given height, holds there until interrupted, then cancels the setpoint.
     *
     * <p>When the setpoint is canceled, the elevator will slide down due to gravity.
     *
     * <p><b>Note: This command is NEVER finished, unless interrupted.</b>
     */
    public Command moveToAndMaintainPosition(Distance elevatorHeightSetpoint) {
        return runEnd(
                () -> this.heightSetpoint = Optional.of(elevatorHeightSetpoint),
                () -> this.heightSetpoint = Optional.empty());
    }

    public Command moveToPosition(Distance elevatorHeightSetpoint) {
        return moveToAndMaintainPosition(elevatorHeightSetpoint).until(this::atReference);
    }

    /**
     * Sets the brake mode of the arm motor.
     *
     * <p><b>CAUTION: it's dangerous to disable motor brake when the robot is enabled, since the elevator will hit the
     * base very hard when the motor is in idle output.</b>
     */
    public void setMotorBrake(boolean brakeModeEnabled) {
        io.setMotorBrake(brakeModeEnabled);
    }
}
