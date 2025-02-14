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
import frc.robot.RobotState;
import frc.robot.utils.AlertsManager;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
    // Hardware interface
    private final ElevatorIO io;
    private final ElevatorInputsAutoLogged inputs;

    // Controllers
    private final ElevatorFeedforward feedforwardController;
    private final PIDController strongFeedbackController;
    private final PIDController weakFeedbackController;
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
    private Distance heightSetpoint;

    public Elevator(ElevatorIO io) {
        this.io = io;
        this.inputs = new ElevatorInputsAutoLogged();

        this.feedforwardController = new ElevatorFeedforward(kS, kG, kV, kA);
        this.strongFeedbackController = new PIDController(kP_STRONG, 0, 0);
        this.weakFeedbackController = new PIDController(kP_WEAK, 0, 0);
        this.profile = new TrapezoidProfile(PROFILE_CONSTRAINS);

        this.elevatorExceedLimitAlert = AlertsManager.create("", Alert.AlertType.kError);
        this.elevatorExceedLimitAlert.set(false);
        this.elevatorHardwareFaultsAlert =
                AlertsManager.create("Elevator hardware faults detected!", Alert.AlertType.kError);
        this.hardwareFaultDebouncer = new Debouncer(0.3, Debouncer.DebounceType.kRising);
        this.elevatorHardwareFaultsAlert.set(false);

        this.currentStateMeters = new TrapezoidProfile.State(0, 0);
        this.heightSetpoint = Meters.zero();

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
        logControlLoops(0, 0, 0);
    }

    private double previousVelocityMPS = 0.0;

    /** Runs the control loops on the elevator to achieve the setpoint. */
    private void executeControlLoops() {
        TrapezoidProfile.State goalState = new TrapezoidProfile.State(heightSetpoint.in(Meters), 0);
        currentStateMeters = profile.calculate(Robot.defaultPeriodSecs, currentStateMeters, goalState);

        double accelerationMPSSq = (currentStateMeters.velocity - previousVelocityMPS) / Robot.defaultPeriodSecs;
        previousVelocityMPS = currentStateMeters.velocity;
        double feedforwardVolts = feedforwardController.calculate(currentStateMeters.velocity, accelerationMPSSq);
        boolean elevatorRequestedToMove =
                Math.abs(currentStateMeters.velocity) > ELEVATOR_MOVING_VELOCITY_THRESHOLD.in(MetersPerSecond);
        PIDController feedbackController = elevatorRequestedToMove ? strongFeedbackController : weakFeedbackController;
        double feedbackVolts = feedbackController.calculate(getHeight().in(Meters), currentStateMeters.position);
        Logger.recordOutput("Elevator/PID/measurement", getHeight().in(Meters));
        Logger.recordOutput("Elevator/PID/setpoint", currentStateMeters.position);

        Voltage output = Volts.of(MathUtil.clamp(
                feedforwardVolts + feedbackVolts, MIN_OUTPUT_VOLTAGE.in(Volts), MAX_OUTPUT_VOLTAGE.in(Volts)));

        if (goalState.position == 0.0 && currentStateMeters.position == 0.0 && atReference()) output = Volts.zero();
        io.setMotorOutput(output);
        logControlLoops(feedforwardVolts, feedbackVolts, output.in(Volts));
    }

    private void logControlLoops(double feedforwardVolts, double feedbackVolts, double outputVolts) {
        Logger.recordOutput("Elevator/PID/Feedforward Volts", feedforwardVolts);
        Logger.recordOutput("Elevator/PID/Feedback Volts", feedbackVolts);
        Logger.recordOutput("Elevator/PID/Requested Output Volts", outputVolts);
    }

    /** @return the measured elevator height, where zero is lowest. */
    public Distance getHeight() {
        // Height = Drum Rotations * Drum Teeth Count * Chain Length
        return CHAN_LENGTH
                .times(inputs.encoderAngle.in(Rotations) / ELEVATOR_GEARING_REDUCTION * ELEVATOR_DRUM_WHEEL_TEETH)
                .times(ELEVATOR_STAGES);
    }

    public Distance getProfileCurrentState() {
        return Meters.of(currentStateMeters.position);
    }

    /** @return the measured elevator velocity, where positive is up. */
    public LinearVelocity getVelocity() {
        return CHAN_LENGTH
                .times(inputs.encoderVelocity.in(RotationsPerSecond)
                        / ELEVATOR_GEARING_REDUCTION
                        * ELEVATOR_DRUM_WHEEL_TEETH)
                .per(Seconds)
                .times(ELEVATOR_STAGES);
    }

    /**
     * Whether the elevator's profile state is close enough to its setpoint.
     *
     * <p>Note that this does not reflect whether the mechanism is actually at its setpoint.
     *
     * @return <code>true</code> if there is a setpoint and the profile state is close enough to it, <code>false</code>
     *     otherwise.
     */
    public boolean atReference() {
        return atReference(this.heightSetpoint);
    }

    public boolean atReference(Distance heightSetpoint) {
        return Math.abs(currentStateMeters.position - heightSetpoint.in(Meters)) < ELEVATOR_PID_TOLERANCE.in(Meters);
    }

    /**
     * Whether the mechanism is actually close enough to its setpoint.
     *
     * @return <code>true</code> if there is a setpoint and the measured elevator height is close enough to the goal,
     *     <code>false</code> otherwise.
     */
    public boolean trulyAtReference() {
        return trulyAtReference(this.heightSetpoint);
    }

    public boolean trulyAtReference(Distance heightSetpoint) {
        return getHeight().minus(heightSetpoint).abs(Meters) < ELEVATOR_PID_TOLERANCE.in(Meters);
    }

    @Override
    public void periodic() {
        // Update inputs from IO and AdvantageKit
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);

        // Run setpoints
        if (DriverStation.isEnabled()) executeControlLoops();
        // Disable PID setpoint if the robot is disabled.
        else executeIdle();

        // Update Alerts
        hardwareFaultDetected = hardwareFaultDebouncer.calculate(!inputs.hardwareConnected);
        elevatorHardwareFaultsAlert.set(hardwareFaultDetected);
        if (getHeight().plus(ELEVATOR_PID_TOLERANCE).lt(Meters.zero())) {
            elevatorExceedLimitAlert.setText(
                    "Elevator height exceeds lower limit: " + getHeight().in(Meters) + " Meters");
            elevatorExceedLimitAlert.set(true);
        } else if (getHeight().minus(ELEVATOR_PID_TOLERANCE).gt(ELEVATOR_MAX_HEIGHT)) {
            elevatorExceedLimitAlert.setText(
                    "Elevator height exceeds higher limit: " + getHeight().in(Meters) + " Meters");
            elevatorExceedLimitAlert.set(true);
        } else elevatorExceedLimitAlert.set(false);

        // Tell drivetrain to lower speed if low speed mode enabled
        RobotState.getInstance().setLowSpeedMode(getHeight().gt(HEIGHT_THRESHOLD_ENABLE_LOW_SPEED_MODE));

        Logger.recordOutput("Elevator/Temperature (Degrees Celsius)", inputs.motorTemperature.in(Celsius));
        Logger.recordOutput("Elevator/Setpoint (Meters)", heightSetpoint.in(Meters));
        Logger.recordOutput("Elevator/Current State Position (Meters)", currentStateMeters.position);
        Logger.recordOutput("Elevator/Measured Height (Meters)", getHeight().in(Meters));
        Logger.recordOutput(
                "Elevator/Measured Velocity (Meters Per Second)", getVelocity().in(MetersPerSecond));
        Logger.recordOutput("Elevator/Current State Velocity (Meters Per Second)", currentStateMeters.velocity);
        Logger.recordOutput("Elevator/At Reference", atReference());
    }

    public void requestElevatorHeight(Distance elevatorHeightSetpoint) {
        this.heightSetpoint = elevatorHeightSetpoint;
    }

    /**
     * Moves to a given height until target is within tolerance.
     *
     * <p><b>Note: This command finishes when the elevator reaches the setpoint, causing the default command to
     * schedule.</b>
     */
    public Command moveToPosition(Distance elevatorHeightSetpoint) {
        return run(() -> requestElevatorHeight(elevatorHeightSetpoint)).until(this::atReference);
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
