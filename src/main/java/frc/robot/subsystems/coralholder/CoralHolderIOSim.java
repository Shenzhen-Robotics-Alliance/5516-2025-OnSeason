package frc.robot.subsystems.coralholder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import java.util.function.Supplier;

public class CoralHolderIOSim implements CoralHolderIO {
    private final Supplier<Pose2d> simulatedDrivetrainPoseSupplier;
    private final Supplier<Angle> armAngleSupplier;
    private final Supplier<Distance> elevatorHeightSupplier;

    private boolean hasCoral;

    public CoralHolderIOSim(
            Supplier<Pose2d> simulatedDrivetrainPoseSupplier,
            Supplier<Angle> armAngleSupplier,
            Supplier<Distance> elevatorHeightSupplier) {
        this.simulatedDrivetrainPoseSupplier = simulatedDrivetrainPoseSupplier;
        this.armAngleSupplier = armAngleSupplier;
        this.elevatorHeightSupplier = elevatorHeightSupplier;
    }

    @Override
    public void updateInputs(CoralHolderInputs inputs) {
        // TODO: finish this class
    }
}
