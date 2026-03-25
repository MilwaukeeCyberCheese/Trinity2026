package edu.msoe.cybercheese.trinity.subsystems.drive;

import choreo.trajectory.SwerveSample;
import edu.msoe.cybercheese.trinity.Constants;
import edu.msoe.cybercheese.trinity.Constants.Mode;
import edu.msoe.cybercheese.trinity.odometry.OdometryCollector;
import edu.msoe.cybercheese.trinity.subsystems.drive.gyro.GyroIO;
import edu.msoe.cybercheese.trinity.subsystems.drive.module.ModuleIO;
import edu.msoe.cybercheese.trinity.subsystems.shooter.ShooterMath;
import edu.msoe.cybercheese.trinity.subsystems.vision.Vision;
import edu.msoe.cybercheese.trinity.util.MathExtras;
import edu.msoe.cybercheese.trinity.util.UnitTypes;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.jspecify.annotations.Nullable;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase implements Vision.VisionConsumer {

    private final OdometryCollector odometryCollector;

    private final @Nullable SwerveDriveSimulation driveSimulation;

    private final GyroIO gyroIo;
    private final GyroIO.GyroInputs gyroInputs = new GyroIO.GyroInputs();
    private final Module[] modules;
    private final SysIdRoutine sysId;
    private final Alert gyroDisconnectedAlert =
            new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(DriveConstants.MODULE_TRANSLATIONS);
    private double rawGyroRotation = 0.0;
    private final SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[] {
        new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()
    };
    private final SwerveDrivePoseEstimator poseEstimator =
            new SwerveDrivePoseEstimator(kinematics, Rotation2d.kZero, lastModulePositions, Pose2d.kZero);

    private final PIDController xController = new PIDController(11.0, 0.0, 0.0);
    private final PIDController yController = new PIDController(11.0, 0.0, 0.0);
    private final PIDController headingController = new PIDController(7.0, 0.0, 0.0);

    private final Field2d field = new Field2d();

    public Drive(
            final @Nullable SwerveDriveSimulation driveSimulation, final GyroIO gyroIo, final ModuleIO[] moduleIos) {
        this.driveSimulation = driveSimulation;

        this.gyroIo = gyroIo;
        this.modules = new Module[moduleIos.length]; // FL, FR, BL, BR
        for (int i = 0; i < moduleIos.length; i++) {
            this.modules[i] = new Module(moduleIos[i], i);
        }

        this.odometryCollector = new OdometryCollector();

        this.odometryCollector.addCallback(this.gyroIo.getOdometryCallback());
        for (final var module : this.modules) {
            this.odometryCollector.addCallback(module.getIo().getOdometryCallback());
        }

        this.headingController.enableContinuousInput(-Math.PI, Math.PI);

        HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

        if (this.driveSimulation != null) {
            SimulatedArena.getInstance().addDriveTrainSimulation(this.driveSimulation);
        }

        this.odometryCollector.start();

        //    AutoBuilder.configure(
        //        this::getPose,
        //        this::setPose,
        //        this::getChassisSpeeds,
        //        this::runVelocity,
        //        new PPHolonomicDriveController(
        //            new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
        //        ppConfig,
        //        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        //        this);
        //    Pathfinding.setPathfinder(new LocalADStarAK());
        //    PathPlannerLogging.setLogActivePathCallback(
        //        (activePath) -> {
        //          Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[0]));
        //        });
        //    PathPlannerLogging.setLogTargetPoseCallback(
        //        (targetPose) -> {
        //          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        //        });

        sysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, null, null, (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism((voltage) -> runCharacterization(voltage.in(UnitTypes.VOLTS)), null, this));

        SmartDashboard.putData("Field", field);
    }

    @Override
    public void periodic() {
        this.odometryCollector.lock();
        try {
            this.gyroIo.updateInputs(this.gyroInputs);
            Logger.processInputs("Drive/Gyro", this.gyroInputs);
            for (var module : modules) {
                module.periodic();
            }

            this.odometryCollector.clearAll();
        } finally {
            this.odometryCollector.unlock();
        }

        // Stop moving when disabled
        if (DriverStation.isDisabled()) {
            for (var module : modules) {
                module.stop();
            }
        }

        // Log empty setpoint states when disabled
        if (DriverStation.isDisabled()) {
            Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
            Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
        }

        // Update odometry
        double[] sampleTimestamps = modules[0].getOdometryTimestamps(); // All signals are sampled together
        int sampleCount = sampleTimestamps.length;
        for (int i = 0; i < sampleCount; i++) {
            // Read wheel positions and deltas from each module
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                modulePositions[moduleIndex] =
                        modules[moduleIndex].getOdometryPositions().get(i);
                moduleDeltas[moduleIndex] = new SwerveModulePosition(
                        modulePositions[moduleIndex].distanceMeters - lastModulePositions[moduleIndex].distanceMeters,
                        modulePositions[moduleIndex].angle);
                lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
            }

            // Update gyro angle
            if (gyroInputs.connected) {
                // Use the real gyro angle
                rawGyroRotation = gyroInputs.odometryYawPositions[i];
            } else {
                // Use the angle delta from the kinematics and module deltas
                Twist2d twist = kinematics.toTwist2d(moduleDeltas);
                rawGyroRotation = rawGyroRotation + twist.dtheta;
            }

            // Apply update
            poseEstimator.updateWithTime(sampleTimestamps[i], Rotation2d.fromRadians(rawGyroRotation), modulePositions);
        }

        // Update gyro alert
        gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.CURRENT_MODE != Mode.SIM);

        field.setRobotPose(getPose());
        SmartDashboard.putNumber("Drive/HubAngle", ShooterMath.absoluteHubAngle(this.getPose()));
    }

    /**
     * Runs the drive at the desired velocity.
     *
     * @param speeds Speeds in meters/sec
     */
    public void runVelocity(ChassisSpeeds speeds) {
        // Calculate module setpoints
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, DriveConstants.maxSpeedMetersPerSecond());

        // Log unoptimized setpoints
        Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

        // Send setpoints to modules
        for (int i = 0; i < 4; i++) {
            modules[i].runSetpoint(setpointStates[i]);
        }

        // Log optimized setpoints (runSetpoint mutates each state)
        Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
    }

    public void runFieldRelativeVelocity(ChassisSpeeds speeds) {
        this.runFieldRelativeVelocity(speeds, false);
    }

    public void runFieldRelativeVelocity(ChassisSpeeds speeds, boolean disableAllianceFlip) {
        this.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds,
                !disableAllianceFlip && MathExtras.isFlipped()
                        ? this.getRotation().plus(new Rotation2d(Math.PI))
                        : this.getRotation()));
    }

    /** Runs the drive in a straight line with the specified drive output. */
    public void runCharacterization(double output) {
        for (int i = 0; i < 4; i++) {
            modules[i].runCharacterization(output);
        }
    }

    /** Stops the drive. */
    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    /**
     * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
     * return to their normal orientations the next time a nonzero velocity is requested.
     */
    public void stopWithX() {
        Rotation2d[] headings = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            headings[i] = DriveConstants.MODULE_TRANSLATIONS[i].getAngle();
        }
        kinematics.resetHeadings(headings);
        stop();
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.quasistatic(direction));
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
    }

    @AutoLogOutput(key = "SwerveStates/Measured")
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getPosition();
        }
        return states;
    }

    @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
    private ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    public double[] getWheelRadiusCharacterizationPositions() {
        double[] values = new double[4];
        for (int i = 0; i < 4; i++) {
            values[i] = modules[i].getWheelRadiusCharacterizationPosition();
        }
        return values;
    }

    public double getFFCharacterizationVelocity() {
        double output = 0.0;
        for (int i = 0; i < 4; i++) {
            output += modules[i].getFFCharacterizationVelocity() / 4.0;
        }
        return output;
    }

    /** Returns the current odometry pose. */
    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        if (this.driveSimulation != null) {
            return this.driveSimulation.getSimulatedDriveTrainPose();
        }

        return poseEstimator.getEstimatedPosition();
    }

    /** Returns the current odometry rotation. */
    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    /** Resets the current odometry pose. */
    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(Rotation2d.fromRadians(rawGyroRotation), getModulePositions(), pose);
    }

    @Override
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
        poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    /** Returns the maximum linear speed in meters per sec. */
    public double getMaxLinearSpeedMetersPerSec() {
        return DriveConstants.maxSpeedMetersPerSecond();
    }

    /** Returns the maximum angular speed in radians per sec. */
    public double getMaxAngularSpeedRadPerSec() {
        return DriveConstants.maxSpeedMetersPerSecond() / DriveConstants.DRIVE_BASE_RADIUS;
    }

    public void followTrajectory(SwerveSample sample) {
        // Get the current pose of the robot
        Pose2d pose = getPose();

        // Generate the next speeds for the robot
        ChassisSpeeds speeds = new ChassisSpeeds(
                sample.vx + xController.calculate(pose.getX(), sample.x),
                sample.vy + yController.calculate(pose.getY(), sample.y),
                sample.omega + headingController.calculate(pose.getRotation().getRadians(), sample.heading));

        // Choreo samples are already in field coordinates and may already be alliance-flipped.
        this.runFieldRelativeVelocity(speeds, true);
    }
}
