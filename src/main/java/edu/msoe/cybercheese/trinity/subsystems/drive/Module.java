package edu.msoe.cybercheese.trinity.subsystems.drive;

import edu.msoe.cybercheese.trinity.subsystems.drive.module.ModuleIO;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Module {
    private final ModuleIO io;
    private final ModuleIO.ModuleInputs inputs = new ModuleIO.ModuleInputs();
    private final int index;

    private final Alert driveDisconnectedAlert;
    private final Alert turnDisconnectedAlert;

    private final List<SwerveModulePosition> odometryPositions = new ArrayList<>();

    public Module(ModuleIO io, int index) {
        this.io = io;
        this.index = index;
        this.driveDisconnectedAlert = new Alert("Disconnected drive motor on module " + index, AlertType.kError);
        this.turnDisconnectedAlert = new Alert("Disconnected turn motor on module " + index, AlertType.kError);
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive/Module/" + index, inputs);

        this.driveDisconnectedAlert.set(!this.inputs.driveConnected);
        this.turnDisconnectedAlert.set(!this.inputs.turnConnected);

        int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
        this.odometryPositions.clear();
        for (int i = 0; i < sampleCount; i++) {
            double positionMeters = this.inputs.odometryDrivePositions[i] * DriveConstants.WHEEL_RADIUS_METERS;
            Rotation2d angle = Rotation2d.fromRadians(this.inputs.odometryTurnPositions[i]);
            this.odometryPositions.add(new SwerveModulePosition(positionMeters, angle));
        }
    }

    public void runSetpoint(SwerveModuleState state) {
        state.optimize(getAngle());
        state.cosineScale(Rotation2d.fromRadians(this.inputs.turnPosition));

        this.io.setDriveVelocity(state.speedMetersPerSecond / DriveConstants.WHEEL_RADIUS_METERS);
        this.io.setTurnPosition(state.angle);
    }

    public void runCharacterization(double output) {
        this.io.setDriveOpenLoop(output);
        this.io.setTurnPosition(Rotation2d.kZero);
    }

    public void stop() {
        this.io.setDriveOpenLoop(0.0);
        this.io.setTurnOpenLoop(0.0);
    }

    public ModuleIO getIo() {
        return io;
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(this.inputs.turnPosition);
    }

    public double getPositionMeters() {
        return this.inputs.drivePosition * DriveConstants.WHEEL_RADIUS_METERS;
    }

    public double getVelocityMetersPerSec() {
        return inputs.driveVelocity * DriveConstants.WHEEL_RADIUS_METERS;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getPositionMeters(), getAngle());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
    }

    public List<SwerveModulePosition> getOdometryPositions() {
        return this.odometryPositions;
    }

    public double[] getOdometryTimestamps() {
        return this.inputs.odometryTimestamps;
    }

    public double getWheelRadiusCharacterizationPosition() {
        return this.inputs.drivePosition;
    }

    public double getFFCharacterizationVelocity() {
        return this.inputs.driveVelocity;
    }
}
