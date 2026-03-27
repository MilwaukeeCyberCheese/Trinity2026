package edu.msoe.cybercheese.trinity;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import com.reduxrobotics.canand.CanandEventLoop;
import edu.msoe.cybercheese.trinity.auto.AutoRoutes;
import edu.msoe.cybercheese.trinity.commands.DriveCommands;
import edu.msoe.cybercheese.trinity.commands.ShooterCommands;
import edu.msoe.cybercheese.trinity.subsystems.drive.*;
import edu.msoe.cybercheese.trinity.subsystems.drive.gyro.GyroIO;
import edu.msoe.cybercheese.trinity.subsystems.drive.gyro.GyroIOCanandGyro;
import edu.msoe.cybercheese.trinity.subsystems.drive.gyro.GyroIOSim;
import edu.msoe.cybercheese.trinity.subsystems.drive.module.ModuleIO;
import edu.msoe.cybercheese.trinity.subsystems.drive.module.ModuleIOSim;
import edu.msoe.cybercheese.trinity.subsystems.drive.module.ModuleIOSpark;
import edu.msoe.cybercheese.trinity.subsystems.hopper.*;
import edu.msoe.cybercheese.trinity.subsystems.intake.*;
import edu.msoe.cybercheese.trinity.subsystems.intakeroller.IntakeRoller;
import edu.msoe.cybercheese.trinity.subsystems.intakeroller.IntakeRollerCommands;
import edu.msoe.cybercheese.trinity.subsystems.intakeroller.IntakeRollerConstants;
import edu.msoe.cybercheese.trinity.subsystems.loader.*;
import edu.msoe.cybercheese.trinity.subsystems.shooter.*;
import edu.msoe.cybercheese.trinity.subsystems.vision.*;
import edu.msoe.cybercheese.trinity.util.MathExtras;
import edu.msoe.cybercheese.trinity.util.hw.MotorConfig;
import edu.msoe.cybercheese.trinity.util.hw.MotorIO;
import edu.msoe.cybercheese.trinity.util.hw.MotorIOSim;
import edu.msoe.cybercheese.trinity.util.hw.MotorIOSpark;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.ArrayList;
import java.util.Objects;
import java.util.concurrent.atomic.AtomicBoolean;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnField;
import org.jspecify.annotations.Nullable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {

    private static final double INTAKE_STOWED_POSITION = 0;
    private static final double INTAKE_DEPLOYED_POSITION = 4.775;
    private static final double INTAKE_MID_POSITION = (INTAKE_STOWED_POSITION + INTAKE_DEPLOYED_POSITION) / 2.0;
    private static final double INTAKE_SIM_EXTENSION_METERS = 0.35;
    private static final double INTAKE_SIM_ROLLER_ACTIVE_THRESHOLD = 1.0;
    private static final double SHOOTING_SIM_LOADER_ACTIVE_THRESHOLD = 1.0;
    private static final double SHOOTING_SIM_HUB_HEIGHT_METERS = 1.6;
    private static final double AUTO_SHOOT_DURATION = 4.0;
    private static final double AUTO_FINAL_SHOOT_EXTRA_DURATION = 5.0;
    private static final double AUTO_POST_SHOT_UNSTICK_DURATION = 0.5;
    private static final double[] INTAKE_POSITIONS = {
        INTAKE_STOWED_POSITION, INTAKE_MID_POSITION, INTAKE_DEPLOYED_POSITION
    };

    private final @Nullable SimulatedArena simulatedArena;
    private final @Nullable SwerveDriveSimulation driveSimulation;
    private final @Nullable IntakeSimulation intakeSimulation;
    private boolean wasShootingInSim = false;

    private final Drive drive;
    private final Vision vision;
    private final Shooter shooter;
    private final Intake intake;
    private final IntakeRoller intakeRoller;
    private final Hopper hopper;
    private final Loader loader;

    private int intakePositionIndex = 0;
    private int savedIntakePositionIndex = 0;

    private final CommandXboxController controller = new CommandXboxController(0);

    private final LoggedDashboardChooser<Command> autoChooser;
    private final AutoFactory autoFactory;

    public RobotContainer() {
        System.out.println("Starting CanandEventLoop...");
        CanandEventLoop.getInstance(); // starts management server for redux alchemist

        System.out.println("Initializing simulatedArena...");
        this.simulatedArena = Constants.CURRENT_MODE == Constants.Mode.SIM ? new Arena2026Rebuilt() : null;
        if (Constants.CURRENT_MODE == Constants.Mode.SIM) {
            SimulatedArena.overrideInstance(Objects.requireNonNull(this.simulatedArena));
            this.simulatedArena.resetFieldForAuto();
        }
        this.driveSimulation = this.simulatedArena != null
                ? new SwerveDriveSimulation(
                        DriveConstants.DRIVE_TRAIN_SIMULATION_CONFIG, new Pose2d(3, 3, Rotation2d.kZero))
                : null;
        this.intakeSimulation = this.driveSimulation != null
                ? IntakeSimulation.OverTheBumperIntake(
                        "Fuel",
                        this.driveSimulation,
                        Units.Meters.of(DriveConstants.TRACK_WIDTH),
                        Units.Meters.of(INTAKE_SIM_EXTENSION_METERS),
                        IntakeSimulation.IntakeSide.FRONT,
                        100)
                : null;

        System.out.println("Initializing swerve IOs...");
        final var moduleIos = new ModuleIO[DriveConstants.MODULE_DEFINITIONS.length];
        for (int i = 0; i < DriveConstants.MODULE_DEFINITIONS.length; i++) {
            moduleIos[i] = this.createModuleIo(
                    DriveConstants.MODULE_DEFINITIONS[i],
                    this.driveSimulation == null ? null : this.driveSimulation.getModules()[i]);
        }
        System.out.println("Initializing Drive...");
        this.drive = new Drive(this.driveSimulation, this.createGyroIo(), moduleIos);

        System.out.println("Initializing Vision...");
        final var cameras = new ArrayList<Camera>();
        for (final var cameraDef : VisionConstants.CAMERA_DEFINITIONS) {
            cameras.add(new Camera(cameraDef, this.createVisionIo(cameraDef, this.drive)));
        }
        this.vision = new Vision(this.drive, cameras);

        System.out.print("Initializing subsystems: shooter...");
        this.shooter = new Shooter(this.createShooterIo());
        System.out.print("ok, Intake...");
        this.intake = new Intake(this.createIntakeIo());
        System.out.print("ok, IntakeRoller...");
        this.intakeRoller = new IntakeRoller(this.createIntakeRollerIo());
        System.out.print("ok, Hopper...");
        this.hopper = new Hopper(this.createHopperIo());
        System.out.print("ok, Loader...");
        this.loader = new Loader(this.createLoaderIo());
        System.out.println("ok!");

        System.out.println("Loading autos...");
        this.autoChooser = new LoggedDashboardChooser<>("Auto Choices", new SendableChooser<>());
        if (Constants.ENABLE_SYSID) {
            this.setupSysIdAutoChooser();
        }

        this.autoFactory = this.setupAutoFactory();

        boolean setDefaultAuto = !Constants.ENABLE_SYSID;
        for (final var path : AutoRoutes.GRAPH.stopTerminatedPaths()) {
            final var label = this.autoPathLabel(path);
            final var command = this.buildAutoPathCommand(path);
            if (setDefaultAuto) {
                this.autoChooser.addDefaultOption(label, command);
                setDefaultAuto = false;
            } else {
                this.autoChooser.addOption(label, command);
            }
        }

        for (final var trajName : Choreo.availableTrajectories()) {
            System.out.println("Loading Trajectory: " + trajName);
            this.autoChooser.addOption(trajName, this.buildSingleTrajectoryAutoCommand(trajName));
        }

        System.out.println("Binding controller...");
        this.configureButtonBindings();
        System.out.println("Done!");
    }

    private AutoFactory setupAutoFactory() {
        AutoFactory autoFactory = new AutoFactory(
                this.drive::getPose, this::logAndSetAutoPose, this.drive::followTrajectory, true, this.drive);

        autoFactory.bind(
                "intakeDeployOn",
                this.dbgLoggedCommand(
                        "intakeDeployOn",
                        Commands.parallel(
                                this.autoSetIntakePositionIndex(INTAKE_POSITIONS.length - 1),
                                IntakeRollerCommands.runVelocity(this.intakeRoller, -270))));
        autoFactory.bind(
                "intakeStowOff",
                this.dbgLoggedCommand(
                        "intakeStowOff",
                        Commands.parallel(
                                this.autoSetIntakePositionIndex(0), IntakeRollerCommands.runVelocity(this.intakeRoller, 0))));

        autoFactory.bind(
                "shootNormal",
                Commands.parallel(
                        ShooterCommands.runVelocity(this.shooter, 430),
                        LoaderCommands.shootWhenReady(this.loader, this.shooter, 80),
                        HopperCommands.runVelocity(this.hopper, () -> -2700)));
        autoFactory.bind(
                "shootHailMary",
                Commands.parallel(
                        ShooterCommands.hailMary(this.shooter),
                        LoaderCommands.runVelocity(this.loader, 80),
                        HopperCommands.runVelocity(this.hopper, () -> -2700)));
        
        return autoFactory;
    }

    private Command autoSetIntakePositionIndex(final int index) {
        return Commands.runOnce(() -> {
            this.intakePositionIndex = index;
            Logger.recordOutput("Auto/Intake/RequestedIndex", this.intakePositionIndex);
            Logger.recordOutput("Auto/Intake/RequestedPosition", INTAKE_POSITIONS[this.intakePositionIndex]);
        });
    }

    private Command dbgLoggedCommand(final String markerName, final Command command) {
        return command.beforeStarting(() -> {
                    System.out.println("START " + markerName);
                })
                .finallyDo(interrupted -> {
                    System.out.println("END " + markerName + " interrupted=" + interrupted);
                });
    }

    private void logAndSetAutoPose(final Pose2d pose) {
        Logger.recordOutput("Auto/ResetPoseRequested", pose);
        Logger.recordOutput("Auto/ResetPoseCurrentBefore", this.drive.getPose());
        Logger.recordOutput("Auto/ResetPoseAllianceKnown", DriverStation.getAlliance().isPresent());
        Logger.recordOutput("Auto/ResetPoseAllianceIsRed", MathExtras.isFlipped());
        this.drive.setPose(pose);
        Logger.recordOutput("Auto/ResetPoseCurrentAfter", this.drive.getPose());
    }

    public void logAllianceDiagnostics() {
        Logger.recordOutput("Auto/AllianceKnown", DriverStation.getAlliance().isPresent());
        Logger.recordOutput("Auto/AllianceIsRed", MathExtras.isFlipped());
    }

    private Command resolveAutoRouteAction(final String actionId) {
        return this.resolveAutoRouteAction(actionId, false);
    }

    private Command resolveAutoRouteAction(final String actionId, final boolean isFinalAction) {
        return switch (actionId) {
            case AutoRoutes.AIM_AND_SHOOT -> this.autoAimAndShoot(isFinalAction
                    ? AUTO_SHOOT_DURATION + AUTO_FINAL_SHOOT_EXTRA_DURATION
                    : AUTO_SHOOT_DURATION);
            default -> throw new IllegalArgumentException("Unknown auto route action: " + actionId);
        };
    }

    private Command autoAimAndShoot() {
        return this.autoAimAndShoot(AUTO_SHOOT_DURATION);
    }

    private Command autoAimAndShoot(final double durationSeconds) {
        return Commands.sequence(
                Commands.deadline(
                        Commands.waitSeconds(durationSeconds),
                        DriveCommands.joystickDriveAtAngle(
                                this.drive,
                                () -> 0.0,
                                () -> 0.0,
                                () -> 0.0,
                                () -> Rotation2d.fromRadians(ShooterMath.absoluteHubAngle(this.drive.getPose())),
                                false),
                        HopperCommands.shootWhenReady(this.hopper, this.shooter, () -> -2700),
                        ShooterCommands.runDefaultedVelocity(this.shooter, this.drive::getPose, 430, () -> false),
                        LoaderCommands.shootWhenReady(this.loader, this.shooter, 80)),
                this.stopAutoShooting());
    }

    private Command stopAutoShooting() {
        return Commands.runOnce(
                () -> {
                    this.shooter.setTargetLocked(false);
                    this.shooter.runVelocity(0);
                    this.loader.runVelocity(0);
                    this.hopper.setVelocity(0);
                },
                this.shooter,
                this.loader,
                this.hopper);
    }

    private Command autoReverseHopperUnstick() {
        return Commands.startEnd(() -> this.hopper.setVelocity(2700), () -> this.hopper.setVelocity(0), this.hopper)
                .withTimeout(AUTO_POST_SHOT_UNSTICK_DURATION);
    }

    private void setupSysIdAutoChooser() {
        this.autoChooser.addOption(
                "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
        this.autoChooser.addOption(
                "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
        this.autoChooser.addOption(
                "Drive SysId (Quasistatic Forward)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        this.autoChooser.addOption(
                "Drive SysId (Quasistatic Reverse)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        this.autoChooser.addOption(
                "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        this.autoChooser.addOption(
                "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }

    private void configureButtonBindings() {
        this.controller
                .back()
                .onTrue(Commands.runOnce(
                                () -> this.drive.setPose(
                                        new Pose2d(
                                                this.drive.getPose().getTranslation(),
                                                MathExtras.isFlipped() ? Rotation2d.kPi : Rotation2d.kZero)),
                                this.drive)
                        .ignoringDisable(true));

        this.drive.setDefaultCommand(DriveCommands.joystickDrive(
                this.drive, 
                () -> this.alterControllerInput(controller.getLeftY()),
                () -> this.alterControllerInput(controller.getLeftX()),
                () -> this.alterControllerInput(controller.getRightX()),
                false));
        this.controller
                .rightBumper()
                .whileTrue(DriveCommands.joystickDriveAtAngle(
                        this.drive,
                        () -> this.alterControllerInput(controller.getLeftY()),
                        () -> this.alterControllerInput(controller.getLeftX()),
                        () -> this.alterControllerInput(controller.getRightX()),
                        () -> Rotation2d.fromRadians(ShooterMath.absoluteHubAngle(this.drive.getPose())),
                        false));
        this.controller.x().or(this.controller.rightStick()).onTrue(Commands.runOnce(this.drive::stopWithX, this.drive));

        this.hopper.setDefaultCommand(HopperCommands.runVelocity(this.hopper, () -> 0));

        // unstick
        this.controller.povLeft().whileTrue(Commands.parallel(
            HopperCommands.runVelocity(this.hopper, () -> 2700),
            LoaderCommands.runVelocity(this.loader, -40)
        ));
        // manual hopper
        this.controller.povRight().whileTrue(Commands.parallel(
            HopperCommands.runVelocity(this.hopper, () -> -2700),
            LoaderCommands.runVelocity(this.loader, 40)
        ));

        this.intake.setDefaultCommand(Commands.run(
                () -> {
                    Logger.recordOutput("Intake/CommandedIndex", this.intakePositionIndex);
                    Logger.recordOutput("Intake/CommandedPosition", INTAKE_POSITIONS[this.intakePositionIndex]);
                    this.intake.setLowerPosition(INTAKE_POSITIONS[this.intakePositionIndex]);
                },
                this.intake));
        // stow
        this.controller.povUp().onTrue(Commands.runOnce(() -> this.intakePositionIndex = 0));
        // lower
        this.controller.povDown().onTrue(Commands.runOnce(() -> this.intakePositionIndex = INTAKE_POSITIONS.length - 1));
        this.controller.a().whileTrue(Commands.startEnd(
                () -> {
                    this.savedIntakePositionIndex = this.intakePositionIndex;
                    this.intakePositionIndex = 0;
                },
                () -> this.intakePositionIndex = this.savedIntakePositionIndex));
        this.controller
                .b()
                .onTrue(Commands.runOnce(
                        () -> this.intakePositionIndex = (this.intakePositionIndex + 1) % INTAKE_POSITIONS.length));

        this.intakeRoller.setDefaultCommand(IntakeRollerCommands.runVelocity(
                this.intakeRoller,
                () -> MathUtil.clamp(-300.0 * this.controller.getLeftTriggerAxis(), -300.0, 0.0)));

        this.shooter.setDefaultCommand(ShooterCommands.runVelocity(this.shooter, 0));
        this.loader.setDefaultCommand(LoaderCommands.runVelocity(this.loader, 0));

        this.controller
                .rightTrigger()
                .whileTrue(Commands.parallel(
                        HopperCommands.shootWhenReady(
                                this.hopper,
                                this.shooter,
                                () -> this.controller.a().getAsBoolean() ? 2700 : -2700),
                        ShooterCommands.runDefaultedVelocity(
                                this.shooter,
                                this.drive::getPose,
                                430,
                                this.controller.rightBumper()), // this works, don't touch!
                        LoaderCommands.shootWhenReady(this.loader, this.shooter, 80)));

        this.controller
                .y()
                .whileTrue(Commands.parallel(
                        ShooterCommands.hailMary(this.shooter), LoaderCommands.runVelocity(this.loader, 80)));
    }

    private double alterControllerInput(double input) {
        final var deadbandedInput = MathUtil.applyDeadband(input, DriveConstants.CONTROLLER_INPUT_DEADBAND);
        double scaled = -deadbandedInput * DriveConstants.JOYSTICK_MULTIPLIER;
        if (this.isSlowDriveEnabled()) {
            scaled *= DriveConstants.SLOW_MULTIPLIER;
        }

        return scaled;
    }

    private boolean isSlowDriveEnabled() {
        return this.controller.leftBumper().getAsBoolean() || this.controller.leftStick().getAsBoolean();
    }

    public Command getAutonomousCommand() {
        return this.autoChooser.get();
    }

    private Command buildSingleTrajectoryAutoCommand(final String trajectoryName) {
        final AutoRoutine routine = this.autoFactory.newRoutine("Trajectory_" + trajectoryName);
        final AutoTrajectory trajectory = routine.trajectory(trajectoryName);
        final AtomicBoolean finished = new AtomicBoolean(false);

        routine.active().onTrue(Commands.sequence(trajectory.resetOdometry(), trajectory.cmd()));
        trajectory.done().onTrue(Commands.runOnce(() -> finished.set(true)));

        return routine.cmd(finished::get);
    }

    private Command buildAutoPathCommand(final java.util.List<String> pathNodeIds) {
        final var path = AutoRoutes.GRAPH.pathOf(pathNodeIds);
        final AutoRoutine routine = this.autoFactory.newRoutine(
                "Path_" + String.join("_", pathNodeIds).replace(' ', '_'));
        final ArrayList<RouteStep> routeSteps = new ArrayList<>();
        final AtomicBoolean finished = new AtomicBoolean(false);

        for (int i = 0; i < path.size(); i++) {
            final var node = path.get(i);
            if (node.routeName() != null) {
                routeSteps.add(new RouteStep(i, node.postRouteActionId(), routine.trajectory(node.routeName())));
            }
        }

        if (routeSteps.isEmpty()) {
            return Commands.none();
        }

        final RouteStep firstStep = routeSteps.get(0);
        routine.active().onTrue(Commands.sequence(firstStep.trajectory().resetOdometry(), firstStep.trajectory().cmd()));

        for (int routeStepIndex = 0; routeStepIndex < routeSteps.size(); routeStepIndex++) {
            final RouteStep currentStep = routeSteps.get(routeStepIndex);
            final boolean hasNextRoute = routeStepIndex + 1 < routeSteps.size();
            final boolean isFinalRoute = !hasNextRoute;
            Command continuation = Commands.none();

            if (currentStep.postRouteActionId() != null) {
                continuation = continuation.andThen(this.resolveAutoRouteAction(currentStep.postRouteActionId(), isFinalRoute));
            }

            if (currentStep.pathIndex() + 1 < path.size()) {
                final var transition = AutoRoutes.GRAPH.transition(
                        path.get(currentStep.pathIndex()).id(), path.get(currentStep.pathIndex() + 1).id());
                if (transition.transitionActionId() != null) {
                    continuation = continuation.andThen(this.resolveAutoRouteAction(transition.transitionActionId()));
                }
            }

            if (hasNextRoute) {
                Command nextTrajectory = routeSteps.get(routeStepIndex + 1).trajectory().cmd();
                if (AutoRoutes.AIM_AND_SHOOT.equals(currentStep.postRouteActionId())) {
                    nextTrajectory = Commands.parallel(nextTrajectory, this.autoReverseHopperUnstick());
                }
                continuation = continuation.andThen(nextTrajectory);
            } else {
                continuation = continuation.andThen(Commands.runOnce(() -> finished.set(true)));
            }

            currentStep.trajectory().done().onTrue(continuation);
        }

        return routine.cmd(finished::get);
    }

    private record RouteStep(int pathIndex, @Nullable String postRouteActionId, AutoTrajectory trajectory) {}

    private String autoPathLabel(final java.util.List<String> pathNodeIds) {
        final var labels = new ArrayList<String>(pathNodeIds);
        labels.add("Stop");
        return String.join(" -> ", labels);
    }

    private VisionIO createVisionIo(final VisionConstants.CameraDefinition definition, final Drive drive) {
        return switch (Constants.CURRENT_MODE) {
            case REAL -> new VisionIOPhotonVision(definition.name(), definition.transform());
            case SIM -> new VisionIOPhotonVisionSim(definition.name(), definition.transform(), drive::getPose);
            case REPLAY -> inputs -> {};
        };
    }

    private GyroIO createGyroIo() {
        return switch (Constants.CURRENT_MODE) {
            case REAL -> new GyroIOCanandGyro();
            case SIM -> new GyroIOSim(
                    Objects.requireNonNull(this.driveSimulation).getGyroSimulation());
            case REPLAY -> inputs -> {};
        };
    }

    private MotorIO createMotorIo(final int canId, final MotorConfig config) {
        return switch (Constants.CURRENT_MODE) {
            case REAL -> new MotorIOSpark(canId, config);
            case SIM -> new MotorIOSim(canId, config);
            case REPLAY -> inputs -> {};
        };
    }

    private ModuleIO createModuleIo(
            final DriveConstants.ModuleDefinition definition, final @Nullable SwerveModuleSimulation moduleSimulation) {
        return switch (Constants.CURRENT_MODE) {
            case REAL -> new ModuleIOSpark(definition);
            case SIM -> new ModuleIOSim(Objects.requireNonNull(moduleSimulation));
            case REPLAY -> inputs -> {};
        };
    }

    private MotorIO createShooterIo() {
        return this.createMotorIo(ShooterConstants.SHOOTER_MOTOR_ID, ShooterConstants.SHOOTER_MOTOR_CONFIG);
    }

    private MotorIO createIntakeIo() {
        return this.createMotorIo(IntakeConstants.LOWER_MOTOR_ID, IntakeConstants.LOWER_MOTOR_CONFIG);
    }

    private MotorIO createIntakeRollerIo() {
        return this.createMotorIo(IntakeRollerConstants.ROLLER_MOTOR_ID, IntakeRollerConstants.ROLLER_MOTOR_CONFIG);
    }

    private MotorIO createHopperIo() {
        return this.createMotorIo(HopperConstants.HOPPER_MOTOR_ID, HopperConstants.HOPPER_MOTOR_CONFIG);
    }

    private MotorIO createLoaderIo() {
        return this.createMotorIo(LoaderConstants.LOADER_MOTOR_ID, LoaderConstants.LOADER_MOTOR_CONFIG);
    }

    public void updateSimulation() {
        if (Constants.CURRENT_MODE != Constants.Mode.SIM) return;

        if (this.intakeSimulation != null) {
            if (this.intake.getLowerPositionSetpoint() <= INTAKE_MID_POSITION
                    && this.intakeRoller.getTargetVelocity() < -INTAKE_SIM_ROLLER_ACTIVE_THRESHOLD) {
                this.intakeSimulation.startIntake();
            } else {
                this.intakeSimulation.stopIntake();
            }

            Logger.recordOutput("FieldSimulation/IntakeFuelCount", this.intakeSimulation.getGamePiecesAmount());
        }
        final boolean shootingInSim = this.shooter.isAimingAtHub()
                && this.shooter.isTargetLocked()
                && this.shooter.isAtSpeed()
                && this.loader.getTargetVelocity() > SHOOTING_SIM_LOADER_ACTIVE_THRESHOLD;
        if (shootingInSim && !this.wasShootingInSim && this.intakeSimulation != null
                && this.intakeSimulation.obtainGamePieceFromIntake()) {
            this.spawnPerfectSimShot(Objects.requireNonNull(this.simulatedArena));
        }
        this.wasShootingInSim = shootingInSim;
        Objects.requireNonNull(this.simulatedArena).simulationPeriodic();
        Logger.recordOutput("FieldSimulation/RobotPosition", this.driveSimulation.getSimulatedDriveTrainPose());
        Logger.recordOutput(
                "FieldSimulation/Fuel", this.simulatedArena.getGamePiecesArrayByType("Fuel"));
    }

    private void spawnPerfectSimShot(final SimulatedArena arena) {
        arena.addGamePieceProjectile(new GamePieceProjectile(
                RebuiltFuelOnField.REBUILT_FUEL_INFO,
                ShooterMath.hubPos(),
                new Translation2d(),
                SHOOTING_SIM_HUB_HEIGHT_METERS,
                0.0,
                new Rotation3d()));
    }
}
