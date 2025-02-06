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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
    // Hardware interface
    private final ArmIO io;
    private final ArmIO.ArmInputs inputs;

    // Controllers
    private final ArmFeedforward armFeedforwardController;
    private final PIDController strongFeedbackController;
    private final PIDController weakFeedbackController;
    private final TrapezoidProfile profile = new TrapezoidProfile(PROFILE_CONSTRAINS);

    // Alerts
    private final Alert armHardwareFaultsAlert;
    private final Alert armNotCalibratedAlert;
    private final Alert armAbsoluteEncoderDisconnectedAlert;

    /** Debounce for hardware faults */
    private final Debouncer hardwareFaultDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kRising);

    /** Whether there is a hardware fault in the motor. */
    private boolean hardwareFaultDetected;

    /** Whether the encoder has been calibrated since booting. */
    private boolean encoderCalibrated;

    /** The offset of the relative encoder (relative - actual) */
    private Rotation2d relativeEncoderOffset;

    /** The current state. */
    private TrapezoidProfile.State currentState;

    /** The desired angle. */
    private Optional<Angle> setpoint;

    public Arm(ArmIO io) {
        this.io = io;
        inputs = new ArmIO.ArmInputs();

        this.armFeedforwardController = new ArmFeedforward(kS, kG, kV, kA);
        this.strongFeedbackController = new PIDController(kP_STRONG, 0, 0);
        this.weakFeedbackController = new PIDController(kP_WEAK, 0, 0);

        this.armHardwareFaultsAlert = new Alert("Arm hardware faults detected!", Alert.AlertType.kError);
        this.armNotCalibratedAlert =
                new Alert("Arm encoder not calibrated! (Arm fault highly likely).", Alert.AlertType.kError);
        this.armAbsoluteEncoderDisconnectedAlert = new Alert(
                "Arm encoder calibrated, but absolute encoder disconnected. (Check it post-match).",
                Alert.AlertType.kWarning);
        this.armHardwareFaultsAlert.set(false);
        this.armNotCalibratedAlert.set(false);
        this.armAbsoluteEncoderDisconnectedAlert.set(false);

        currentState = new TrapezoidProfile.State(ARM_UPPER_LIMIT.in(Radians), 0);
        setpoint = Optional.empty();

        hardwareFaultDetected = false;
        encoderCalibrated = false;

        // Calibrate the relative encoder.
        // Assumes that the arm starts at upper limit during boot.
        // The calibration will be overwritten if the absolute encoder is available.
        // Offset = Relative Angle - Actual Angle, where Relative Angle is 0 at boot.
        relativeEncoderOffset = Rotation2d.kZero.minus(new Rotation2d(ARM_UPPER_LIMIT));

        io.setMotorBrake(true);
    }

    /**
     * <p>Calibrates the relative encoder offset using the absolute encoder reading.</p>
     * */
    private void calibrateEncoders(Rotation2d absoluteEncoderAngle) {
        relativeEncoderOffset = new Rotation2d(inputs.relativeMechanismAngle).minus(absoluteEncoderAngle);
        encoderCalibrated = true;
    }

    /**
     * <p>Sets the arm motor to idle.</p>
     * <p><b>Note: this does not unlock the motors if they are set to brake mode.</b></p>
     * */
    private void executeIdle() {
        io.setMotorOutput(Volts.zero());
        currentState = new TrapezoidProfile.State(getArmAngle().getRadians(), 0);
        previousVelocity = 0.0;
    }

    private double previousVelocity = 0.0;
    private static final double ARM_MOVING_VELOCITY_THRESHOLD_RAD_PER_SEC = Math.toRadians(5);

    /**
     * <p>Runs the control loops on the arm to achieve the setpoint.</p>
     * */
    private void executeControlLoops(Angle setpoint) {
        TrapezoidProfile.State goalState = new TrapezoidProfile.State(setpoint.in(Radians), 0);
        currentState = profile.calculate(Robot.defaultPeriodSecs, currentState, goalState);

        double accelerationRadPerSecSq = (currentState.velocity - previousVelocity) / Robot.defaultPeriodSecs;
        double feedforwardVolts = armFeedforwardController.calculate(
                getArmAngle().getRadians(), currentState.velocity, accelerationRadPerSecSq);
        boolean armRequestedToMove = Math.abs(currentState.velocity) > ARM_MOVING_VELOCITY_THRESHOLD_RAD_PER_SEC;
        PIDController feedbackController = armRequestedToMove ? strongFeedbackController : weakFeedbackController;
        double feedbackVolts = feedbackController.calculate(getArmAngle().getRadians(), currentState.position);

        Voltage voltage = Volts.of(MathUtil.clamp(feedforwardVolts + feedbackVolts, -ARM_MAX_VOLTS, ARM_MAX_VOLTS));
        io.setMotorOutput(voltage);
    }

    @Override
    public void periodic() {
        // Update inputs from IO and AdvantageKit.
        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);

        // Calibrates encoder if needed when the robot is disabled and the motor is connected.
        if (!encoderCalibrated && DriverStation.isDisabled() && inputs.motorConnected)
            inputs.absoluteEncoderAngle.ifPresent(this::calibrateEncoders);

        // Disable PID setpoint if the robot is disabled.
        if (DriverStation.isDisabled()) setpoint = Optional.empty();
        // Run setpoints (or run idle if no setpoint).
        setpoint.ifPresentOrElse(this::executeControlLoops, this::executeIdle);

        // Update Alerts
        if (hardwareFaultDetected) encoderCalibrated = false;
        hardwareFaultDetected = hardwareFaultDebouncer.calculate(!inputs.motorConnected);
        armHardwareFaultsAlert.set(hardwareFaultDetected);
        armNotCalibratedAlert.set(!encoderCalibrated);
        armAbsoluteEncoderDisconnectedAlert.set(inputs.absoluteEncoderAngle.isEmpty());
    }

    /**
     * <p>Whether the arm is close enough to its setpoint.</p>
     * @return <code>true</code> if there is a setpoint and the arm is close enough to it, <code>false</code> otherwise.
     *  */
    public boolean atReference() {
        if (setpoint.isEmpty())
            return false;

        double errorRad = getArmAngle().minus(new Rotation2d(setpoint.get())).getRadians();
        return Math.abs(errorRad) < ARM_PID_TOLERANCE.in(Radians);
    }

    /**
     * <p>Moves to a given angle, holds there until interrupted, then cancels the setpoint.</p>
     * <p><b>Note: This command is NEVER finished, unless interrupted.</b></p>
     * */
    public Command moveToAndMaintainPosition(Angle armAngleSetpoint) {
        return runEnd(
                () -> this.setpoint = Optional.of(armAngleSetpoint),
                () -> this.setpoint = Optional.empty());
    }

    /**
     * <p>Moves to a given angle, cancels the setpoint when reached.</p>
     * <p>When brake mode is on, the arm will still hold at that angle.</p>
     * <p><b>Note: This command finishes automatically when the setpoint is reached, causing the default command to schedule.</b></p>
     * */
    public Command moveToPosition(Angle armAngleSetpoint) {
        return Commands.runOnce(() -> this.setpoint = Optional.of(armAngleSetpoint))
                .andThen(Commands.waitUntil(() -> false));
    }

    /**
     * @return the measured arm angle, where zero is horizontally forward.
     * */
    public Rotation2d getArmAngle() {
        return new Rotation2d(inputs.relativeMechanismAngle)
                .minus(relativeEncoderOffset);
    }

    /**
     * <p>Sets the brake mode of the arm motor.</p>
     * */
    public void setBrakeMode(boolean brakeModeEnabled) {
        io.setMotorBrake(brakeModeEnabled);
    }
}
