package frc.robot.constants;

import static edu.wpi.first.units.Units.Centimeters;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Robot;
import frc.robot.commands.reefscape.ReefAlignment;

public class ReefConstants {
    public static void loadStatic() {
        ReefAlignment.BranchTarget target1 = REEF_ALIGNMENT_POSITIONS_BLUE[0];
        ReefAlignment.BranchTarget target2 = REEF_ALIGNMENT_POSITIONS_RED[0];
    }

    private static final Distance ROBOT_TO_TARGET_DISTANCE = Robot.CURRENT_ROBOT_MODE == RobotMode.SIM
            // for simulation
            ? Centimeters.of(37.5)
            // for real robot (measure this on field)
            // "AdvantageKit/RealOutputs/RobotToSelectedBranchTarget" - X Axis
            : Centimeters.of(38.0);

    // "AdvantageKit/RealOutputs/RobotToSelectedBranchTarget" - Y Axis - Take Absolute Value
    public static final ReefAlignment.BranchTarget[] REEF_ALIGNMENT_POSITIONS_BLUE = new ReefAlignment.BranchTarget[] {
        ReefAlignment.BranchTarget.measured(18, false, ROBOT_TO_TARGET_DISTANCE, Centimeters.of(17.0)),
        ReefAlignment.BranchTarget.measured(18, true, ROBOT_TO_TARGET_DISTANCE, Centimeters.of(17.0)),
        ReefAlignment.BranchTarget.measured(17, false, ROBOT_TO_TARGET_DISTANCE, Centimeters.of(17.0)),
        ReefAlignment.BranchTarget.measured(17, true, ROBOT_TO_TARGET_DISTANCE, Centimeters.of(17.0)),
        ReefAlignment.BranchTarget.measured(22, false, ROBOT_TO_TARGET_DISTANCE, Centimeters.of(17.0)),
        ReefAlignment.BranchTarget.measured(22, true, ROBOT_TO_TARGET_DISTANCE, Centimeters.of(17.0)),
        ReefAlignment.BranchTarget.measured(21, false, ROBOT_TO_TARGET_DISTANCE, Centimeters.of(17.0)),
        ReefAlignment.BranchTarget.measured(21, true, ROBOT_TO_TARGET_DISTANCE, Centimeters.of(17.0)),
        ReefAlignment.BranchTarget.measured(20, false, ROBOT_TO_TARGET_DISTANCE, Centimeters.of(17.0)),
        ReefAlignment.BranchTarget.measured(20, true, ROBOT_TO_TARGET_DISTANCE, Centimeters.of(17.0)),
        ReefAlignment.BranchTarget.measured(19, false, ROBOT_TO_TARGET_DISTANCE, Centimeters.of(17.0)),
        ReefAlignment.BranchTarget.measured(19, true, ROBOT_TO_TARGET_DISTANCE, Centimeters.of(17.0)),
    };

    public static final ReefAlignment.BranchTarget[] REEF_ALIGNMENT_POSITIONS_RED = new ReefAlignment.BranchTarget[] {
        ReefAlignment.BranchTarget.measured(7, false, ROBOT_TO_TARGET_DISTANCE, Centimeters.of(17.0)),
        ReefAlignment.BranchTarget.measured(7, true, ROBOT_TO_TARGET_DISTANCE, Centimeters.of(17.0)),
        ReefAlignment.BranchTarget.measured(8, false, ROBOT_TO_TARGET_DISTANCE, Centimeters.of(17.0)),
        ReefAlignment.BranchTarget.measured(8, true, ROBOT_TO_TARGET_DISTANCE, Centimeters.of(17.0)),
        ReefAlignment.BranchTarget.measured(9, false, ROBOT_TO_TARGET_DISTANCE, Centimeters.of(17.0)),
        ReefAlignment.BranchTarget.measured(9, true, ROBOT_TO_TARGET_DISTANCE, Centimeters.of(17.0)),
        ReefAlignment.BranchTarget.measured(10, false, ROBOT_TO_TARGET_DISTANCE, Centimeters.of(17.0)),
        ReefAlignment.BranchTarget.measured(10, true, ROBOT_TO_TARGET_DISTANCE, Centimeters.of(17.0)),
        ReefAlignment.BranchTarget.measured(11, false, ROBOT_TO_TARGET_DISTANCE, Centimeters.of(17.0)),
        ReefAlignment.BranchTarget.measured(11, true, ROBOT_TO_TARGET_DISTANCE, Centimeters.of(17.0)),
        ReefAlignment.BranchTarget.measured(6, false, ROBOT_TO_TARGET_DISTANCE, Centimeters.of(17.0)),
        ReefAlignment.BranchTarget.measured(6, true, ROBOT_TO_TARGET_DISTANCE, Centimeters.of(17.0)),
    };
}
