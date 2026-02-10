package edu.msoe.cybercheese.trinity.subsystems.drive.module;

import edu.msoe.cybercheese.trinity.subsystems.drive.DriveConstants;
import edu.msoe.cybercheese.trinity.util.SparkUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import java.util.Arrays;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

/** Physics sim implementation of module IO. */
public class ModuleIOSim implements ModuleIO {

    private final SwerveModuleSimulation moduleSimulation;
    private final SimulatedMotorController.GenericMotorController driveMotor;
    private final SimulatedMotorController.GenericMotorController turnMotor;

    private boolean driveClosedLoop = false;
    private boolean turnClosedLoop = false;
    private PIDController driveController =
            new PIDController(DriveConstants.DRIVE_SIM_P, 0, DriveConstants.DRIVE_SIM_D);
    private PIDController turnController = new PIDController(DriveConstants.TURN_SIM_P, 0, DriveConstants.TURN_SIM_D);
    private double driveFFVolts = 0.0;
    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;

    public ModuleIOSim(final SwerveModuleSimulation moduleSimulation) {
        this.moduleSimulation = moduleSimulation;
        this.driveMotor = this.moduleSimulation
                .useGenericMotorControllerForDrive()
                .withCurrentLimit(Units.Amps.of(DriveConstants.DRIVE_MOTOR_CURRENT_LIMIT));
        this.turnMotor = this.moduleSimulation
                .useGenericControllerForSteer()
                .withCurrentLimit(Units.Amps.of(DriveConstants.TURN_MOTOR_CURRENT_LIMIT));

        this.turnController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void updateInputs(ModuleInputs inputs) {
        if (this.driveClosedLoop) {
            this.driveAppliedVolts = this.driveFFVolts
                    + this.driveController.calculate(
                            this.moduleSimulation.getDriveWheelFinalSpeed().in(Units.RadiansPerSecond));
        } else {
            this.driveController.reset();
        }

        if (this.turnClosedLoop) {
            this.turnAppliedVolts = this.turnController.calculate(
                    this.moduleSimulation.getSteerAbsoluteFacing().getRadians());
        } else {
            this.turnController.reset();
        }

        // Update simulation state
        this.driveMotor.requestVoltage(Units.Volts.of(this.driveAppliedVolts));
        this.turnMotor.requestVoltage(Units.Volts.of(this.turnAppliedVolts));

        inputs.driveConnected = true;
        inputs.drivePosition =
                this.moduleSimulation.getDriveWheelFinalPosition().in(Units.Radians);
        inputs.driveVelocity = this.moduleSimulation.getDriveWheelFinalSpeed().in(Units.RadiansPerSecond);
        inputs.driveAppliedVolts = this.driveAppliedVolts;
        inputs.driveCurrentAmps =
                Math.abs(this.moduleSimulation.getDriveMotorStatorCurrent().in(Units.Amps));

        inputs.turnConnected = true;
        inputs.turnPosition = this.moduleSimulation.getSteerAbsoluteAngle().in(Units.Radians);
        inputs.turnVelocity =
                this.moduleSimulation.getSteerAbsoluteEncoderSpeed().in(Units.RadiansPerSecond);
        inputs.turnAppliedVolts = this.turnAppliedVolts;
        inputs.turnCurrentAmps =
                Math.abs(this.moduleSimulation.getSteerMotorStatorCurrent().in(Units.Amps));

        inputs.odometryTimestamps = SparkUtil.getSimulationOdometryTimeStamps();
        inputs.odometryDrivePositions = Arrays.stream(this.moduleSimulation.getCachedDriveWheelFinalPositions())
                .mapToDouble(a -> a.in(Units.Radians))
                .toArray();
        inputs.odometryTurnPositions = Arrays.stream(this.moduleSimulation.getCachedSteerAbsolutePositions())
                .mapToDouble(Rotation2d::getRadians)
                .toArray();
    }

    @Override
    public void setDriveOpenLoop(double output) {
        driveClosedLoop = false;
        driveAppliedVolts = output;
    }

    @Override
    public void setTurnOpenLoop(double output) {
        turnClosedLoop = false;
        turnAppliedVolts = output;
    }

    @Override
    public void setDriveVelocity(double velocityRadPerSec) {
        driveClosedLoop = true;
        driveFFVolts = DriveConstants.DRIVE_SIM_KS * Math.signum(velocityRadPerSec)
                + DriveConstants.DRIVE_SIM_KV * velocityRadPerSec;
        driveController.setSetpoint(velocityRadPerSec);
    }

    @Override
    public void setTurnPosition(Rotation2d rotation) {
        turnClosedLoop = true;
        turnController.setSetpoint(rotation.getRadians());
    }
}
