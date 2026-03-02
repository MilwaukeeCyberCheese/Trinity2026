package edu.msoe.cybercheese.trinity.subsystems.drive.module;

import edu.msoe.cybercheese.trinity.subsystems.drive.DriveConstants;
import edu.msoe.cybercheese.trinity.util.SparkUtil;
import edu.msoe.cybercheese.trinity.util.UnitTypes;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
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
    private final PIDController driveController;
    private final PIDController turnController =
            new PIDController(DriveConstants.TURN_SIM_P, 0, DriveConstants.TURN_SIM_D);
    private double driveFFVolts = 0.0;
    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;

    public ModuleIOSim(final SwerveModuleSimulation moduleSimulation) {
        this.moduleSimulation = moduleSimulation;
        this.driveMotor = this.moduleSimulation
                .useGenericMotorControllerForDrive()
                .withCurrentLimit(UnitTypes.AMPS.of(DriveConstants.DRIVE_MOTOR_CONFIG.currentLimit()));
        this.turnMotor = this.moduleSimulation
                .useGenericControllerForSteer()
                .withCurrentLimit(UnitTypes.AMPS.of(DriveConstants.TURN_MOTOR_CURRENT_LIMIT));

        this.driveController =
                new PIDController(DriveConstants.DRIVE_MOTOR_CONFIG.p(), 0, DriveConstants.DRIVE_MOTOR_CONFIG.d());

        this.turnController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void updateInputs(ModuleInputs inputs) {
        if (this.driveClosedLoop) {
            this.driveAppliedVolts = this.driveFFVolts
                    + this.driveController.calculate(
                            this.moduleSimulation.getDriveWheelFinalSpeed().in(UnitTypes.RADIANS_PER_SECOND));
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
        this.driveMotor.requestVoltage(UnitTypes.VOLTS.of(this.driveAppliedVolts));
        this.turnMotor.requestVoltage(UnitTypes.VOLTS.of(this.turnAppliedVolts));

        inputs.drive.connected = true;
        inputs.drive.position =
                this.moduleSimulation.getDriveWheelFinalPosition().in(UnitTypes.RADIANS);
        inputs.drive.velocity = this.moduleSimulation.getDriveWheelFinalSpeed().in(UnitTypes.RADIANS_PER_SECOND);
        inputs.drive.appliedVolts = this.driveAppliedVolts;
        inputs.drive.currentAmps =
                Math.abs(this.moduleSimulation.getDriveMotorStatorCurrent().in(UnitTypes.AMPS));

        inputs.turnConnected = true;
        inputs.turnPosition = this.moduleSimulation.getSteerAbsoluteAngle().in(UnitTypes.RADIANS);
        inputs.turnVelocity =
                this.moduleSimulation.getSteerAbsoluteEncoderSpeed().in(UnitTypes.RADIANS_PER_SECOND);
        inputs.turnAppliedVolts = this.turnAppliedVolts;
        inputs.turnCurrentAmps =
                Math.abs(this.moduleSimulation.getSteerMotorStatorCurrent().in(UnitTypes.AMPS));

        inputs.odometryTimestamps = SparkUtil.getSimulationOdometryTimeStamps();
        inputs.odometryDrivePositions = Arrays.stream(this.moduleSimulation.getCachedDriveWheelFinalPositions())
                .mapToDouble(a -> a.in(UnitTypes.RADIANS))
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
        driveFFVolts = DriveConstants.DRIVE_MOTOR_CONFIG.simKS() * Math.signum(velocityRadPerSec)
                + DriveConstants.DRIVE_MOTOR_CONFIG.simKV() * velocityRadPerSec;
        driveController.setSetpoint(velocityRadPerSec);
    }

    @Override
    public void setTurnPosition(Rotation2d rotation) {
        turnClosedLoop = true;
        turnController.setSetpoint(rotation.getRadians());
    }
}
