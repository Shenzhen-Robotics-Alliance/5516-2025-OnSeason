package frc.robot.subsystems.vision.apriltags;

import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.constants.LogPaths.*;
import static frc.robot.constants.VisionConstants.*;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotState;
import frc.robot.utils.AlertsManager;
import java.util.List;
import java.util.Optional;
import java.util.OptionalInt;
import org.littletonrobotics.junction.Logger;

public class AprilTagVision extends SubsystemBase {
    private final AprilTagVisionIO io;
    private final AprilTagVisionIO.CameraInputs[] inputs;

    private final MapleMultiTagPoseEstimator multiTagPoseEstimator;
    private final Alert[] camerasDisconnectedAlerts;
    private final Alert[] camerasNoResultAlerts;
    private final Debouncer[] camerasNoResultDebouncer;

    private final LinearFilter visionHasResultAverage = LinearFilter.movingAverage(50);

    public AprilTagVision(AprilTagVisionIO io, List<PhotonCameraProperties> camerasProperties) {
        this.io = io;
        this.inputs = new AprilTagVisionIO.CameraInputs[camerasProperties.size()];
        for (int i = 0; i < inputs.length; i++) inputs[i] = new AprilTagVisionIO.CameraInputs(i);
        this.camerasDisconnectedAlerts = new Alert[camerasProperties.size()];
        this.camerasNoResultAlerts = new Alert[camerasProperties.size()];
        this.camerasNoResultDebouncer = new Debouncer[camerasProperties.size()];
        for (int i = 0; i < camerasProperties.size(); i++) {
            this.camerasDisconnectedAlerts[i] = AlertsManager.create(
                    "Photon Camera " + i + " '" + camerasProperties.get(i).name + "' disconnected",
                    Alert.AlertType.kError);
            this.camerasNoResultAlerts[i] = AlertsManager.create(
                    "Photon Camera " + i + " '" + camerasProperties.get(i).name + "' no result",
                    Alert.AlertType.kWarning);
            this.camerasNoResultDebouncer[i] = new Debouncer(0.5);
            this.camerasDisconnectedAlerts[i].set(false);
        }

        this.multiTagPoseEstimator = new MapleMultiTagPoseEstimator(
                fieldLayout, new CameraHeightAndPitchRollAngleFilter(), camerasProperties);
    }

    private Optional<MapleMultiTagPoseEstimator.VisionObservation> result = Optional.empty();

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        for (int i = 0; i < inputs.length; i++) Logger.processInputs(APRIL_TAGS_VISION_PATH + "Camera_" + i, inputs[i]);

        for (int i = 0; i < inputs.length; i++) {
            this.camerasDisconnectedAlerts[i].set(!inputs[i].cameraConnected);
            this.camerasNoResultAlerts[i].set((!camerasDisconnectedAlerts[i].get())
                    && camerasNoResultDebouncer[i].calculate(!inputs[i].newPipeLineResultAvailable));
        }

        result = multiTagPoseEstimator.estimateRobotPose(inputs, getResultsTimeStamp());
        result.ifPresent(RobotState.getInstance()::addVisionObservation);
        RobotState.getInstance().visionObservationRate =
                visionHasResultAverage.calculate(result.isPresent() ? 1.0 : 0.0);

        Logger.recordOutput(
                APRIL_TAGS_VISION_PATH + "Results/Estimated Pose", displayVisionPointEstimateResult(result));
        SmartDashboard.putBoolean("Vision Result Trustable", resultPresent);
        Logger.recordOutput(APRIL_TAGS_VISION_PATH + "Results/Presented", resultPresent);
    }

    private static final Pose2d EMPTY_DISPLAY = new Pose2d(-114514, -114514, new Rotation2d());
    private Optional<MapleMultiTagPoseEstimator.VisionObservation> previousResult = Optional.empty();
    private final Debouncer resultPresentDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kFalling);
    private boolean resultPresent = false;

    private Pose2d displayVisionPointEstimateResult(Optional<MapleMultiTagPoseEstimator.VisionObservation> result) {
        resultPresent = resultPresentDebouncer.calculate(result.isPresent());
        if (!resultPresent) return EMPTY_DISPLAY;

        Pose2d toReturn = result.orElse(
                        previousResult.orElse(new MapleMultiTagPoseEstimator.VisionObservation(EMPTY_DISPLAY, null, 0)))
                .visionPose();
        result.ifPresent(newResult -> previousResult = Optional.of(newResult));
        return toReturn;
    }

    private double getResultsTimeStamp() {
        if (inputs.length == 0) return Timer.getTimestamp();
        double totalTimeStampSeconds = 0, camerasUsed = 0;
        for (AprilTagVisionIO.CameraInputs input : inputs) {
            if (input.newPipeLineResultAvailable) {
                totalTimeStampSeconds += input.timeStampSeconds;
                camerasUsed++;
            }
        }
        return totalTimeStampSeconds / camerasUsed - ADDITIONAL_LATENCY_COMPENSATION.in(Seconds);
    }

    public Command focusOnTarget(int tagId, int cameraToFocusId) {
        return startEnd(
                () -> multiTagPoseEstimator.enableFocusMode(tagId, cameraToFocusId),
                multiTagPoseEstimator::disableFocusMode);
    }

    public Command focusOnTarget(OptionalInt tagId, Integer... cameraToFocusId) {
        return startEnd(
                () -> multiTagPoseEstimator.setFocusMode(tagId, cameraToFocusId),
                multiTagPoseEstimator::disableFocusMode);
    }

    public final Trigger cameraDisconnected = new Trigger(() -> {
        for (AprilTagVisionIO.CameraInputs input : AprilTagVision.this.inputs) if (!input.cameraConnected) return true;
        return false;
    });
}
