package frc.robot.subsystems.superstructure.elevator;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.superstructure.elevator.ElevatorConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
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
    private double heightSetpointMeters;

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
        this.heightSetpointMeters = 0.0;

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
        io.setMotorOutput(0.0);
        currentStateMeters = new TrapezoidProfile.State(getHeightMeters(), 0);
        previousVelocityMPS = getMeasuredVelocityMPS();
        logControlLoops(0, 0, 0);
    }

    private double previousVelocityMPS = 0.0;

    /** Runs the control loops on the elevator to achieve the setpoint. */
    private void executeControlLoops() {
        TrapezoidProfile.State goalState = new TrapezoidProfile.State(heightSetpointMeters, 0);
        currentStateMeters = profile.calculate(Robot.defaultPeriodSecs, currentStateMeters, goalState);

        double accelerationMPSSq = (currentStateMeters.velocity - previousVelocityMPS) / Robot.defaultPeriodSecs;
        previousVelocityMPS = currentStateMeters.velocity;
        double feedforwardVolts = feedforwardController.calculate(currentStateMeters.velocity, accelerationMPSSq);
        boolean elevatorRequestedToMove =
                Math.abs(currentStateMeters.velocity) > ELEVATOR_MOVING_VELOCITY_THRESHOLD.in(MetersPerSecond);
        PIDController feedbackController = elevatorRequestedToMove ? strongFeedbackController : weakFeedbackController;
        double feedbackVolts = feedbackController.calculate(getHeightMeters(), currentStateMeters.position);
        Logger.recordOutput("Elevator/PID/measurement", getHeightMeters());
        Logger.recordOutput("Elevator/PID/setpoint", currentStateMeters.position);

        double outputVolts = MathUtil.clamp(
                feedforwardVolts + feedbackVolts, MIN_OUTPUT_VOLTAGE.in(Volts), MAX_OUTPUT_VOLTAGE.in(Volts));

        if (goalState.position == 0.0 && currentStateMeters.position == 0.0 && atReference()) outputVolts = 0.0;
        io.setMotorOutput(outputVolts);
        logControlLoops(feedforwardVolts, feedbackVolts, outputVolts);
    }

    private void logControlLoops(double feedforwardVolts, double feedbackVolts, double outputVolts) {
        Logger.recordOutput("Elevator/PID/Feedforward Volts", feedforwardVolts);
        Logger.recordOutput("Elevator/PID/Feedback Volts", feedbackVolts);
        Logger.recordOutput("Elevator/PID/Requested Output Volts", outputVolts);
    }

    /** @return the measured elevator height, where zero is lowest. */
    private static final double CHAIN_LENGTH_METERS = CHAIN_LENGTH.in(Meters);

    public double getHeightMeters() {
        // Height = Drum Rotations * Drum Teeth Count * Chain Length
        return CHAIN_LENGTH_METERS
                * Units.radiansToRotations(inputs.encoderAngleRad)
                / ELEVATOR_GEARING_REDUCTION
                * ELEVATOR_DRUM_WHEEL_TEETH
                * ELEVATOR_STAGES;
    }

    public double getProfileCurrentStateMeters() {
        return currentStateMeters.position;
    }

    /** @return the measured elevator velocity, where positive is up. */
    public double getMeasuredVelocityMPS() {
        return CHAIN_LENGTH_METERS
                * Units.radiansToRotations(inputs.encoderVelocityRadPerSec)
                / ELEVATOR_GEARING_REDUCTION
                * ELEVATOR_DRUM_WHEEL_TEETH
                * ELEVATOR_STAGES;
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
        return atReference(this.heightSetpointMeters);
    }

    public boolean atReference(double heightSetpointMeters) {
        return Math.abs(currentStateMeters.position - heightSetpointMeters) < ELEVATOR_PID_TOLERANCE.in(Meters);
    }

    /**
     * Whether the mechanism is actually close enough to its setpoint.
     *
     * @return <code>true</code> if there is a setpoint and the measured elevator height is close enough to the goal,
     *     <code>false</code> otherwise.
     */
    public boolean trulyAtReference() {
        return trulyAtReference(this.heightSetpointMeters);
    }

    private static final double PID_TOLERANCE = ELEVATOR_PID_TOLERANCE.in(Meters);
    private static final double MAX_HEIGHT_METERS = ELEVATOR_MAX_HEIGHT.in(Meters);

    public boolean trulyAtReference(double heightSetpointMeters) {
        return Math.abs(getHeightMeters() - heightSetpointMeters) < PID_TOLERANCE;
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
        if (getHeightMeters() < -PID_TOLERANCE) {
            elevatorExceedLimitAlert.setText("Elevator height exceeds lower limit: " + getHeightMeters() + " Meters");
            elevatorExceedLimitAlert.set(true);
        } else if (getHeightMeters() > MAX_HEIGHT_METERS + PID_TOLERANCE) {
            elevatorExceedLimitAlert.setText("Elevator height exceeds higher limit: " + getHeightMeters() + " Meters");
            elevatorExceedLimitAlert.set(true);
        } else elevatorExceedLimitAlert.set(false);

        // Tell drivetrain to lower speed if low speed mode enabled
        RobotState.getInstance().setLowSpeedMode(getHeightMeters() > HEIGHT_THRESHOLD_ENABLE_LOW_SPEED_MODE.in(Meters));

        Logger.recordOutput("Elevator/Setpoint (Meters)", heightSetpointMeters);
        Logger.recordOutput("Elevator/Current State Position (Meters)", currentStateMeters.position);
        Logger.recordOutput("Elevator/Measured Height (Meters)", getHeightMeters());
        Logger.recordOutput("Elevator/Measured Velocity (Meters Per Second)", getMeasuredVelocityMPS());
        Logger.recordOutput("Elevator/Current State Velocity (Meters Per Second)", currentStateMeters.velocity);
        Logger.recordOutput("Elevator/At Reference", atReference());
    }

    public void requestElevatorHeight(double elevatorHeightSetpointMeters) {
        this.heightSetpointMeters = elevatorHeightSetpointMeters;
    }

    /**
     * Moves to a given height until target is within tolerance.
     *
     * <p><b>Note: This command finishes when the elevator reaches the setpoint, causing the default command to
     * schedule.</b>
     */
    public Command moveToPosition(double elevatorHeightSetpoint) {
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
