package frc.robot.subsystems.superstructure.arm;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.superstructure.arm.ArmConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.superstructure.SuperStructure;
import frc.robot.utils.AlertsManager;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
    // Hardware interface
    private final ArmIO io;
    private final ArmIO.ArmInputs inputs;

    // Controllers
    private final ArmFeedforward feedforwardController;
    private final PIDController feedbackController;
    private final TrapezoidProfile profile;

    // Alerts
    private final Alert armHardwareFaultsAlert;
    private final Alert armNotCalibratedAlert;
    private final Alert armAbsoluteEncoderDisconnectedAlert;

    /** Debounce for hardware faults */
    private final Debouncer hardwareFaultDebouncer;

    /** Whether there is a hardware fault in the motor. */
    private boolean hardwareFaultDetected;

    /** Whether the encoder has been calibrated since booting. */
    private boolean encoderCalibrated;

    /** The offset of the relative encoder (relative - actual) */
    private Rotation2d relativeEncoderOffset;

    /** The current state. */
    private TrapezoidProfile.State currentStateRad;

    /** The desired angle. */
    private Angle setpoint;

    public Arm(ArmIO io) {
        this.io = io;
        inputs = new ArmIO.ArmInputs();

        this.feedforwardController =
                new ArmFeedforward(PID_CONSTANTS.kS(), PID_CONSTANTS.kG(), PID_CONSTANTS.kV(), PID_CONSTANTS.kA());
        this.feedbackController = new PIDController(PID_CONSTANTS.kP(), 0, 0);
        this.profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
                PID_CONSTANTS.VELOCITY_CONSTRAIN().in(RadiansPerSecond),
                PID_CONSTANTS.ACCELERATION_CONSTRAIN().in(RadiansPerSecondPerSecond)));

        this.armHardwareFaultsAlert = AlertsManager.create("Arm hardware faults detected!", Alert.AlertType.kError);
        this.armNotCalibratedAlert =
                AlertsManager.create("Arm encoder not calibrated! (Arm fault highly likely).", Alert.AlertType.kError);
        this.armAbsoluteEncoderDisconnectedAlert = AlertsManager.create(
                "Arm encoder calibrated, but absolute encoder disconnected. (Check it post-match).",
                Alert.AlertType.kWarning);
        this.hardwareFaultDebouncer = new Debouncer(0.3, Debouncer.DebounceType.kRising);
        this.armHardwareFaultsAlert.set(false);
        this.armNotCalibratedAlert.set(false);
        this.armAbsoluteEncoderDisconnectedAlert.set(false);

        currentStateRad = new TrapezoidProfile.State(
                HARDWARE_CONSTANTS.ARM_UPPER_HARD_LIMIT().in(Radians), 0);
        setpoint = SuperStructure.SuperStructurePose.IDLE.armAngle;

        hardwareFaultDetected = false;
        encoderCalibrated = false;

        // Calibrate the relative encoder.
        // Assumes that the arm starts at upper limit during boot.
        // The calibration will be overwritten if the absolute encoder is available.
        // Offset = Relative Angle - Actual Angle, where Relative Angle is 0 at boot.
        relativeEncoderOffset = Rotation2d.kZero.minus(new Rotation2d((HARDWARE_CONSTANTS.ARM_UPPER_HARD_LIMIT())));

        io.setMotorBrake(true);
    }

    /** Calibrates the relative encoder offset using the absolute encoder reading. */
    private void calibrateEncoders(Rotation2d absoluteEncoderAngle) {
        // real = relative - offset
        // so
        // offset = relative - real
        relativeEncoderOffset = Rotation2d.fromRadians(
                        inputs.relativeEncoderAngleRad / HARDWARE_CONSTANTS.ARM_GEARING_REDUCTION())
                .minus(absoluteEncoderAngle);
        encoderCalibrated = true;
    }

    /**
     * Sets the arm motor to idle.
     *
     * <p><b>Note: this does not unlock the motors if they are set to brake mode.</b>
     */
    private void executeIdle() {
        io.setMotorOutput(Volts.zero());
        currentStateRad = new TrapezoidProfile.State(getArmAngle().getRadians(), 0);
        previousVelocityRadPerSec = 0.0;
        logControlLoops(0, 0, 0);
    }

    private double previousVelocityRadPerSec = 0.0;

    /** Runs the control loops on the arm to achieve the setpoint. */
    private void executeControlLoops(double dtSeconds) {
        dtSeconds = MathUtil.clamp(dtSeconds, 0, 0.1);

        TrapezoidProfile.State goalState = new TrapezoidProfile.State(setpoint.in(Radians), 0);
        currentStateRad = profile.calculate(dtSeconds, currentStateRad, goalState);

        double accelerationRadPerSecSq =
                (currentStateRad.velocity - previousVelocityRadPerSec) / Robot.defaultPeriodSecs;
        previousVelocityRadPerSec = currentStateRad.velocity;
        double feedforwardVolts = feedforwardController.calculate(
                getArmAngle().getRadians(), currentStateRad.velocity, accelerationRadPerSecSq);
        double feedbackVolts = feedbackController.calculate(getArmAngle().getRadians(), currentStateRad.position);

        Voltage output = Volts.of(MathUtil.clamp(
                feedforwardVolts + feedbackVolts, -ARM_MAX_VOLTAGE.in(Volts), ARM_MAX_VOLTAGE.in(Volts)));
        io.setMotorOutput(output);
        logControlLoops(feedforwardVolts, feedbackVolts, output.in(Volts));
    }

    private void logControlLoops(double feedforwardVolts, double feedbackVolts, double outputVolts) {
        Logger.recordOutput("Arm/PID/Feedforward Volts", feedforwardVolts);
        Logger.recordOutput("Arm/PID/Feedback Volts", feedbackVolts);
        Logger.recordOutput("Arm/PID/Requested Output Volts", outputVolts);
    }

    private double previousTimeSeconds = Timer.getTimestamp();

    @Override
    public void periodic() {
        // Update inputs from IO and AdvantageKit.
        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);

        executeControlLoops(Timer.getTimestamp() - previousTimeSeconds);
        if (DriverStation.isDisabled()) executeIdle();
        previousTimeSeconds = Timer.getTimestamp();

        // Calibrates encoder if needed when the robot is disabled and the motor is connected.
        if (!encoderCalibrated && DriverStation.isDisabled() && inputs.motorConnected)
            inputs.absoluteEncoderAngle.ifPresent(this::calibrateEncoders);

        // Update Alerts
        hardwareFaultDetected = hardwareFaultDebouncer.calculate(!inputs.motorConnected);
        if (hardwareFaultDetected) encoderCalibrated = false;
        armHardwareFaultsAlert.set(hardwareFaultDetected);
        armNotCalibratedAlert.set(!encoderCalibrated);
        armAbsoluteEncoderDisconnectedAlert.set(encoderCalibrated && inputs.absoluteEncoderAngle.isEmpty());

        Logger.recordOutput("Arm/Setpoint (Degrees)", setpoint.in(Degrees));
        Logger.recordOutput("Arm/Current State Position(Degrees)", Math.toDegrees(currentStateRad.position));
        Logger.recordOutput("Arm/MeasuredAngle (Degrees)", getArmAngle().getDegrees());
        Logger.recordOutput(
                "Arm/Current State Velocity (Degrees Per Second)", Math.toDegrees(currentStateRad.velocity));
        Logger.recordOutput("Arm/Measured Velocity (Degrees Per Second)", Math.toDegrees(getVelocityRadPerSec()));
        Logger.recordOutput("Arm/At Reference", atReference());
    }

    /**
     * Whether the arm's profile state is close enough to its setpoint.
     *
     * <p>Note that this does not reflect whether the mechanism is actually at its setpoint.
     *
     * @return <code>true</code> if there is a setpoint and the profile is close enough to the goal, <code>false</code>
     *     otherwise.
     */
    public boolean atReference() {
        return atReference(this.setpoint);
    }

    public boolean atReference(Angle setpoint) {
        double errorRad = Rotation2d.fromRadians(currentStateRad.position)
                .minus(new Rotation2d(setpoint))
                .getRadians();
        return Math.abs(errorRad) < PID_CONSTANTS.TOLERANCE().in(Radians);
    }

    /**
     * Whether the mechanism is actually close enough to its setpoint.
     *
     * @return <code>true</code> if there is a setpoint and the measured arm angle is close enough to the goal, <code>
     *     false</code> otherwise.
     */
    public boolean trulyAtReference() {
        return trulyAtReference(this.setpoint);
    }

    public boolean trulyAtReference(Angle setpoint) {
        double errorRad = getArmAngle().minus(new Rotation2d(setpoint)).getRadians();
        return Math.abs(errorRad) < PID_CONSTANTS.TOLERANCE().in(Radians);
    }

    /** Request the arm to move to a given setpoint. */
    public void requestPosition(Angle setpoint) {
        this.setpoint = setpoint;
    }

    /**
     * Moves to a given angle, cancels the setpoint when reached.
     *
     * <p>When brake mode is on, the arm will still hold at that angle.
     *
     * <p><b>Note: This command finishes automatically when the setpoint is reached, causing the default command to
     * schedule.</b>
     */
    public Command moveToPosition(Angle setpoint) {
        return run(() -> requestPosition(setpoint)).until(this::atReference);
    }

    /** @return the measured arm angle, where zero is horizontally forward. */
    public Rotation2d getArmAngle() {
        return Rotation2d.fromRadians(inputs.relativeEncoderAngleRad / HARDWARE_CONSTANTS.ARM_GEARING_REDUCTION())
                .minus(relativeEncoderOffset);
    }

    public double getVelocityRadPerSec() {
        return inputs.encoderVelocityRadPerSec / HARDWARE_CONSTANTS.ARM_GEARING_REDUCTION();
    }

    public Rotation2d getProfileCurrentState() {
        return Rotation2d.fromRadians(currentStateRad.position);
    }

    /** Sets the brake mode of the arm motor. */
    public void setMotorBrake(boolean brakeModeEnabled) {
        io.setMotorBrake(brakeModeEnabled);
    }
}
