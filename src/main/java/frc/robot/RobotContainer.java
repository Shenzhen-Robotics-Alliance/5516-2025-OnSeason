// Original Source:
// https://github.com/Mechanical-Advantage/AdvantageKit/tree/main/example_projects/advanced_swerve_drive/src/main,
// Copyright 2021-2024 FRC 6328
// Modified by 5516 Iron Maple https://github.com/Shenzhen-Robotics-Alliance/

package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.autos.*;
import frc.robot.commands.drive.*;
import frc.robot.commands.reefscape.FaceCoralStation;
import frc.robot.commands.reefscape.ReefAlignment;
import frc.robot.constants.*;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.coralholder.CoralHolder;
import frc.robot.subsystems.coralholder.CoralHolderIOReal;
import frc.robot.subsystems.coralholder.CoralHolderIOSim;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.drive.IO.*;
import frc.robot.subsystems.led.LEDAnimation;
import frc.robot.subsystems.led.LEDStatusLight;
import frc.robot.subsystems.superstructure.SuperStructure;
import frc.robot.subsystems.superstructure.SuperStructureVisualizer;
import frc.robot.subsystems.superstructure.arm.Arm;
import frc.robot.subsystems.superstructure.arm.ArmIOReal;
import frc.robot.subsystems.superstructure.arm.ArmIOSim;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOReal;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOSim;
import frc.robot.subsystems.vision.apriltags.AprilTagVision;
import frc.robot.subsystems.vision.apriltags.AprilTagVisionIOReal;
import frc.robot.subsystems.vision.apriltags.ApriltagVisionIOSim;
import frc.robot.subsystems.vision.apriltags.PhotonCameraProperties;
import frc.robot.utils.AlertsManager;
import frc.robot.utils.MapleJoystickDriveInput;
import java.util.*;
import java.util.function.IntSupplier;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.ironmaple.utils.FieldMirroringUtils;
import org.ironmaple.utils.mathutils.MapleCommonMath;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    public static final boolean SIMULATE_AUTO_PLACEMENT_INACCURACY = true;

    // pdp for akit logging
    public final LoggedPowerDistribution powerDistribution;
    // Subsystems
    public final SwerveDrive drive;
    public final AprilTagVision aprilTagVision;
    public final LEDStatusLight ledStatusLight;

    // Controller
    // public final DriverMap driver = new DriverMap.LeftHandedPS5(0);
    public final DriverMap driver = new DriverMap.LeftHandedXbox(0);
    public final CommandXboxController operator = new CommandXboxController(1);

    private final LoggedDashboardChooser<Auto> autoChooser;
    private final SendableChooser<Supplier<Command>> testChooser;

    // Simulated drive
    private final SwerveDriveSimulation driveSimulation;
    public final Arm arm;
    public final Elevator elevator;
    public final SuperStructure superStructure;
    public final CoralHolder coralHolder;
    private final Climb climb;

    private final Field2d field = new Field2d();

    public final Trigger isAlgaeMode;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        final List<PhotonCameraProperties> camerasProperties =
                VisionConstants.photonVisionCameras; // load configs stored directly in VisionConstants.java

        switch (Robot.CURRENT_ROBOT_MODE) {
            case REAL -> {
                // Real robot, instantiate hardware IO implementations
                driveSimulation = null;

                powerDistribution = LoggedPowerDistribution.getInstance(1, PowerDistribution.ModuleType.kRev);

                /* CTRE Chassis: */
                drive = new SwerveDrive(
                        SwerveDrive.DriveType.CTRE_TIME_SYNCHRONIZED,
                        new GyroIOPigeon2(TunerConstants.DrivetrainConstants, false),
                        new CanBusIOReal(TunerConstants.kCANBus),
                        new ModuleIOTalon(TunerConstants.FrontLeft, "FrontLeft"),
                        new ModuleIOTalon(TunerConstants.FrontRight, "FrontRight"),
                        new ModuleIOTalon(TunerConstants.BackLeft, "BackLeft"),
                        new ModuleIOTalon(TunerConstants.BackRight, "BackRight"));

                aprilTagVision = new AprilTagVision(new AprilTagVisionIOReal(camerasProperties), camerasProperties);

                arm = new Arm(new ArmIOReal());
                elevator = new Elevator(new ElevatorIOReal());
                coralHolder = new CoralHolder(
                        new CoralHolderIOReal(),
                        RobotState.getInstance()::getPrimaryEstimatorPose,
                        arm::getArmAngle,
                        elevator::getHeightMeters);
                climb = new Climb(new Climb.ClimbIOReal());
            }

            case SIM -> {
                SimulatedArena.overrideSimulationTimings(
                        Seconds.of(Robot.defaultPeriodSecs), DriveTrainConstants.SIMULATION_TICKS_IN_1_PERIOD);
                this.driveSimulation = new SwerveDriveSimulation(
                        DriveTrainSimulationConfig.Default()
                                .withRobotMass(DriveTrainConstants.ROBOT_MASS)
                                .withBumperSize(DriveTrainConstants.BUMPER_LENGTH, DriveTrainConstants.BUMPER_WIDTH)
                                .withTrackLengthTrackWidth(
                                        DriveTrainConstants.TRACK_LENGTH, DriveTrainConstants.TRACK_WIDTH)
                                .withSwerveModule(new SwerveModuleSimulationConfig(
                                        DriveTrainConstants.DRIVE_MOTOR_MODEL,
                                        DriveTrainConstants.STEER_MOTOR_MODEL,
                                        DriveTrainConstants.DRIVE_GEAR_RATIO,
                                        DriveTrainConstants.STEER_GEAR_RATIO,
                                        DriveTrainConstants.DRIVE_FRICTION_VOLTAGE,
                                        DriveTrainConstants.STEER_FRICTION_VOLTAGE,
                                        DriveTrainConstants.WHEEL_RADIUS,
                                        DriveTrainConstants.STEER_INERTIA,
                                        DriveTrainConstants.WHEEL_COEFFICIENT_OF_FRICTION))
                                .withGyro(DriveTrainConstants.gyroSimulationFactory),
                        new Pose2d(3, 3, new Rotation2d()));
                SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);

                powerDistribution = LoggedPowerDistribution.getInstance();
                // Sim robot, instantiate physics sim IO implementations
                final ModuleIOSim frontLeft = new ModuleIOSim(driveSimulation.getModules()[0]),
                        frontRight = new ModuleIOSim(driveSimulation.getModules()[1]),
                        backLeft = new ModuleIOSim(driveSimulation.getModules()[2]),
                        backRight = new ModuleIOSim(driveSimulation.getModules()[3]);
                final GyroIOSim gyroIOSim = new GyroIOSim(driveSimulation.getGyroSimulation());
                drive = new SwerveDrive(
                        SwerveDrive.DriveType.GENERIC,
                        gyroIOSim,
                        (canBusInputs) -> {},
                        frontLeft,
                        frontRight,
                        backLeft,
                        backRight);

                aprilTagVision = new AprilTagVision(
                        new ApriltagVisionIOSim(
                                camerasProperties,
                                VisionConstants.fieldLayout,
                                driveSimulation::getSimulatedDriveTrainPose),
                        camerasProperties);

                SimulatedArena.getInstance().resetFieldForAuto();

                arm = new Arm(new ArmIOSim());
                elevator = new Elevator(new ElevatorIOSim());
                coralHolder = new CoralHolder(
                        new CoralHolderIOSim(driveSimulation, arm::getArmAngle, elevator::getHeightMeters),
                        driveSimulation::getSimulatedDriveTrainPose,
                        arm::getArmAngle,
                        elevator::getHeightMeters);

                climb = new Climb(new Climb.ClimbIO() {});
            }

            default -> {
                this.driveSimulation = null;

                powerDistribution = LoggedPowerDistribution.getInstance();
                // Replayed robot, disable IO implementations
                drive = new SwerveDrive(
                        SwerveDrive.DriveType.GENERIC,
                        (canBusInputs) -> {},
                        (inputs) -> {},
                        (inputs) -> {},
                        (inputs) -> {},
                        (inputs) -> {},
                        (inputs) -> {});

                aprilTagVision = new AprilTagVision((inputs) -> {}, camerasProperties);

                arm = new Arm(armInputs -> {});
                elevator = new Elevator(elevatorInputs -> {});
                coralHolder = new CoralHolder(
                        coralHolderInputs -> {},
                        RobotState.getInstance()::getPrimaryEstimatorPose,
                        arm::getArmAngle,
                        elevator::getHeightMeters);

                climb = new Climb(new Climb.ClimbIO() {});
            }
        }

        this.superStructure = new SuperStructure(elevator, arm);
        this.ledStatusLight = new LEDStatusLight(0, 63, true, false);

        this.drive.configHolonomicPathPlannerAutoBuilder(field);

        SmartDashboard.putData("Select Test", testChooser = buildTestsChooser());
        autoChooser = buildAutoChooser();

        isAlgaeMode = new Trigger(() -> superStructure.currentPose() == SuperStructure.SuperStructurePose.LOW_ALGAE
                || superStructure.currentPose() == SuperStructure.SuperStructurePose.HIGH_ALGAE
                || superStructure.currentPose() == SuperStructure.SuperStructurePose.ALGAE_SWAP_2
                || superStructure.currentPose() == SuperStructure.SuperStructurePose.SCORE_ALGAE
                || superStructure.targetPose() == SuperStructure.SuperStructurePose.LOW_ALGAE
                || superStructure.targetPose() == SuperStructure.SuperStructurePose.HIGH_ALGAE
                || superStructure.targetPose() == SuperStructure.SuperStructurePose.ALGAE_SWAP_2
                || superStructure.targetPose() == SuperStructure.SuperStructurePose.SCORE_ALGAE);
        configureButtonBindings();
        configureLEDEffects();

        SmartDashboard.putData("Field", field);

        setMotorBrake(true);
    }

    private void configureAutoTriggers(PathPlannerAuto pathPlannerAuto) {}

    private LoggedDashboardChooser<Auto> buildAutoChooser() {
        final LoggedDashboardChooser<Auto> autoSendableChooser = new LoggedDashboardChooser<>("Select Auto");
        autoSendableChooser.addDefaultOption("None", Auto.none());
        autoSendableChooser.addOption("[Four Coral - Standard] <-- LEFT SIDE <-- ", new FourCoralStandard(false));
        autoSendableChooser.addOption("[Four Coral - Standard] --> RIGHT SIDE -->", new FourCoralStandard(true));

        SmartDashboard.putData("Select Auto", autoSendableChooser.getSendableChooser());
        return autoSendableChooser;
    }

    private SendableChooser<Supplier<Command>> buildTestsChooser() {
        final SendableChooser<Supplier<Command>> testsChooser = new SendableChooser<>();
        testsChooser.setDefaultOption("None", Commands::none);
        testsChooser.addOption(
                "Drive SysId- Quasistatic - Forward", () -> drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        testsChooser.addOption(
                "Drive SysId- Quasistatic - Reverse", () -> drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        testsChooser.addOption(
                "Drive SysId- Dynamic - Forward", () -> drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        testsChooser.addOption(
                "Drive SysId- Dynamic - Reverse", () -> drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        return testsChooser;
    }

    private boolean isDSPresentedAsRed = FieldMirroringUtils.isSidePresentedAsRed();
    private Command autonomousCommand = Commands.none();
    private Auto previouslySelectedAuto = Auto.none();
    /** reconfigures button bindings if alliance station has changed re-create autos if not yet created */
    public void checkForCommandChanges() {
        final Auto selectedAuto = autoChooser.get();
        if (FieldMirroringUtils.isSidePresentedAsRed() == isDSPresentedAsRed & selectedAuto == previouslySelectedAuto)
            return;

        try {
            this.autonomousCommand = selectedAuto.getAutoCommand(this);
            configureAutoTriggers(new PathPlannerAuto(autonomousCommand, selectedAuto.getStartingPoseAtBlueAlliance()));
        } catch (Exception e) {
            this.autonomousCommand = Commands.none();
            DriverStation.reportError(
                    "Error Occurred while obtaining autonomous command: \n"
                            + e.getMessage()
                            + "\n"
                            + Arrays.toString(e.getStackTrace()),
                    false);
            throw new RuntimeException(e);
        }
        resetFieldAndOdometryForAuto(selectedAuto.getStartingPoseAtBlueAlliance());

        previouslySelectedAuto = selectedAuto;
        isDSPresentedAsRed = FieldMirroringUtils.isSidePresentedAsRed();
    }

    private void resetFieldAndOdometryForAuto(Pose2d robotStartingPoseAtBlueAlliance) {
        final Pose2d startingPose = FieldMirroringUtils.toCurrentAlliancePose(robotStartingPoseAtBlueAlliance);

        if (driveSimulation != null) {
            Transform2d placementError = SIMULATE_AUTO_PLACEMENT_INACCURACY
                    ? new Transform2d(
                            MapleCommonMath.generateRandomNormal(0, 0.2),
                            MapleCommonMath.generateRandomNormal(0, 0.2),
                            Rotation2d.fromDegrees(MapleCommonMath.generateRandomNormal(0, 1)))
                    : new Transform2d();
            driveSimulation.setSimulationWorldPose(startingPose.plus(placementError));
            SimulatedArena.getInstance().resetFieldForAuto();
        }

        aprilTagVision
                .focusOnTarget(-1, -1)
                .withTimeout(0.1)
                .alongWith(Commands.runOnce(() -> drive.setPose(startingPose), drive))
                .ignoringDisable(true)
                .schedule();
    }

    public Command scoreCoral(double scoringTimeOut) {
        return Commands.deferredProxy(() -> superStructure
                .moveToPose(SuperStructure.SuperStructurePose.SCORE_L4_COMPLETE)
                .onlyIf(() -> superStructure.targetPose() == SuperStructure.SuperStructurePose.SCORE_L4)
                .alongWith(coralHolder.scoreCoral(scoringTimeOut)));
    }

    public Command moveToL4() {
        Command shuffleCoralDuringElevatorMovement = Commands.waitSeconds(0.3)
                .deadlineFor(coralHolder.moveCoralToPlace())
                .andThen(coralHolder.keepCoralShuffledForever());
        return superStructure
                .moveToPose(SuperStructure.SuperStructurePose.SCORE_L4)
                .deadlineFor(shuffleCoralDuringElevatorMovement.onlyIf(coralHolder.hasCoral))
                .asProxy();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by instantiating a
     * {@link GenericHID} or one of its subclasses ({@link Joystick} or {@link XboxController}), and then passing it to
     * a {@link JoystickButton}.
     */
    public void configureButtonBindings() {
        SmartDashboard.putData(
                "Enable Motor Brake",
                Commands.runOnce(() -> setMotorBrake(true)).ignoringDisable(true));
        SmartDashboard.putData(
                "Disable Motor Brake",
                Commands.runOnce(() -> setMotorBrake(false)).ignoringDisable(true));

        /* joystick drive command */
        final MapleJoystickDriveInput driveInput = driver.getDriveInput();
        IntSupplier pov =
                // driver.getController().getHID()::getPOV;
                () -> -1;
        final JoystickDrive joystickDrive = new JoystickDrive(driveInput, () -> true, pov, drive);
        drive.setDefaultCommand(joystickDrive.ignoringDisable(true));
        JoystickDrive.instance = Optional.of(joystickDrive);

        /* reset gyro heading manually (in case the vision does not work) */
        driver.resetOdometryButton()
                .onTrue(Commands.runOnce(
                                () -> drive.setPose(new Pose2d(
                                        drive.getPose().getTranslation(),
                                        FieldMirroringUtils.getCurrentAllianceDriverStationFacing())),
                                drive)
                        .ignoringDisable(true));

        /* lock chassis with x-formation */
        driver.lockChassisWithXFormatButton().whileTrue(drive.lockChassisWithXFormation());

        /* auto alignment example, delete it for your project */
        driver.autoAlignmentButtonLeft()
                .whileTrue(ReefAlignment.alignToNearestBranch(drive, aprilTagVision, ledStatusLight, false));
        driver.autoAlignmentButtonRight()
                .whileTrue(ReefAlignment.alignToNearestBranch(drive, aprilTagVision, ledStatusLight, true));

        coralHolder.setDefaultCommand(coralHolder.runIdle());

        Command flashLEDForIntake =
                ledStatusLight.playAnimationPeriodically(new LEDAnimation.Charging(Color.kPurple), 4);
        driver.intakeButton()
                .onTrue(superStructure.moveToPose(SuperStructure.SuperStructurePose.IDLE))
                .whileTrue(Commands.sequence(
                                Commands.waitUntil(
                                        () -> superStructure.currentPose() == SuperStructure.SuperStructurePose.IDLE),
                                coralHolder.intakeCoralSequence().beforeStarting(flashLEDForIntake::schedule))
                        .finallyDo(flashLEDForIntake::cancel))
                // move coral in place before retrieving arm
                .onFalse(coralHolder.intakeCoralSequence().onlyIf(coralHolder.hasCoral));
        driver.l2Button()
                .onTrue(superStructure.moveToPose(SuperStructure.SuperStructurePose.SCORE_L2))
                .onTrue(coralHolder.keepCoralShuffledForever());
        driver.l3Button()
                .onTrue(superStructure.moveToPose(SuperStructure.SuperStructurePose.SCORE_L3))
                .onTrue(coralHolder.keepCoralShuffledForever());
        driver.l4Button().onTrue(moveToL4());

        driver.autoRotationButton()
                .whileTrue(Commands.either(
                        JoystickDriveAndAimAtTarget.driveAndAimAtTarget(
                                driveInput,
                                drive,
                                () -> FieldMirroringUtils.toCurrentAllianceTranslation(ReefAlignment.REEF_CENTER_BLUE),
                                null,
                                JoystickConfigs.DEFAULT_TRANSLATIONAL_SENSITIVITY,
                                false),
                        FaceCoralStation.faceCoralStation(drive, driveInput),
                        coralHolder.hasCoral));

        // Retrieve elevator at the start of teleop
        new Trigger(DriverStation::isTeleopEnabled)
                .onTrue(superStructure.moveToPose(SuperStructure.SuperStructurePose.IDLE));

        // Retrieve elevator when robot is about to tip
        drive.driveTrainTipping
                .and(isAlgaeMode.negate())
                .onTrue(superStructure.retrieveElevator())
                .onTrue(ledStatusLight
                        .playAnimation(new LEDAnimation.Breathe(() -> Color.kRed), 0.25, 4)
                        .ignoringDisable(true));

        driver.scoreButton()
                .onTrue(scoreCoral(1.0))
                .onFalse(superStructure.moveToPose(SuperStructure.SuperStructurePose.IDLE));

        operator.povDown()
                .and(operator.leftBumper().or(isAlgaeMode))
                .onTrue(superStructure.moveToPose(SuperStructure.SuperStructurePose.LOW_ALGAE));
        operator.povUp()
                .and(operator.leftBumper().or(isAlgaeMode))
                .onTrue(superStructure.moveToPose(SuperStructure.SuperStructurePose.HIGH_ALGAE));
        operator.rightBumper()
                .and(isAlgaeMode)
                .onTrue(superStructure.moveToPose(SuperStructure.SuperStructurePose.SCORE_ALGAE));
        operator.leftTrigger(0.5).and(isAlgaeMode).onTrue(coralHolder.runVolts(-3, 0));
        operator.rightTrigger(0.5).and(isAlgaeMode).whileTrue(coralHolder.runVolts(6, 0));
        isAlgaeMode.onFalse(coralHolder.runVolts(-1.5, 0).withTimeout(0.5));
        operator.back().whileTrue(coralHolder.runVolts(-0.5, -6));

        // climbing
        operator.start().and(operator.b()).onTrue(climb.climbCommand(operator::getLeftY));
        operator.start().and(operator.x()).onTrue(climb.cancelClimb());

        operator.y().onTrue(ReefAlignment.selectReefPartButton(3).ignoringDisable(true));
        operator.a().onTrue(ReefAlignment.selectReefPartButton(0).ignoringDisable(true));
        operator.x().whileTrue(ReefAlignment.lefterTargetButton(0.3).ignoringDisable(true));
        operator.b().whileTrue(ReefAlignment.righterTargetButton(0.3).ignoringDisable(true));
    }

    public void configureLEDEffects() {
        ledStatusLight.setDefaultCommand(ledStatusLight.showRobotState());
        coralHolder.hasCoral.onTrue(
                ledStatusLight.playAnimation(new LEDAnimation.Breathe(() -> Color.kYellow), 0.2, 4));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autonomousCommand;
    }

    public Command getTestCommand() {
        return testChooser.getSelected().get();
    }

    public void updateFieldSimAndDisplay() {
        if (driveSimulation == null) return;

        SimulatedArena.getInstance().simulationPeriodic();
        Logger.recordOutput("FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
        Logger.recordOutput(
                "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
        Logger.recordOutput(
                "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
    }

    private final Alert autoPlacementIncorrect = AlertsManager.create(
            "Expected Autonomous robot placement position does not match reality, IS THE SELECTED AUTO CORRECT?",
            Alert.AlertType.kWarning);
    private static final double AUTO_PLACEMENT_TOLERANCE_METERS = 0.25;
    private static final double AUTO_PLACEMENT_TOLERANCE_DEGREES = 5;

    public void updateTelemetryAndLED() {
        field.setRobotPose(
                Robot.CURRENT_ROBOT_MODE == RobotMode.SIM
                        ? driveSimulation.getSimulatedDriveTrainPose()
                        : drive.getPose());
        if (Robot.CURRENT_ROBOT_MODE == RobotMode.SIM)
            field.getObject("Odometry").setPose(drive.getPose());

        ReefAlignment.updateDashboard();

        SuperStructureVisualizer.visualizeMechanisms(
                "measuredMechanismPoses", elevator.getHeightMeters(), arm.getArmAngle());
        SuperStructureVisualizer.visualizeMechanisms(
                "profileCurrentStatePoses", elevator.getProfileCurrentStateMeters(), arm.getProfileCurrentState());
        Logger.recordOutput("SuperStructure/currentPose", superStructure.currentPose());

        Pose2d autoStartingPose =
                FieldMirroringUtils.toCurrentAlliancePose(previouslySelectedAuto.getStartingPoseAtBlueAlliance());
        Pose2d currentPose = RobotState.getInstance().getVisionPose();
        Transform2d difference = autoStartingPose.minus(currentPose);
        boolean autoPlacementIncorrectDetected = difference.getTranslation().getNorm() > AUTO_PLACEMENT_TOLERANCE_METERS
                || Math.abs(difference.getRotation().getDegrees()) > AUTO_PLACEMENT_TOLERANCE_DEGREES;
        // autoPlacementIncorrect.set(autoPlacementIncorrectDetected && DriverStation.isDisabled());
        autoPlacementIncorrect.set(false);

        AlertsManager.updateLEDAndLog(ledStatusLight);
    }

    public static boolean motorBrakeEnabled = false;

    public void setMotorBrake(boolean brakeModeEnabled) {
        if (motorBrakeEnabled == brakeModeEnabled) return;

        System.out.println("Set motor brake: " + brakeModeEnabled);
        drive.setMotorBrake(brakeModeEnabled);
        arm.setMotorBrake(brakeModeEnabled);
        elevator.setMotorBrake(brakeModeEnabled);

        motorBrakeEnabled = brakeModeEnabled;
    }
}
