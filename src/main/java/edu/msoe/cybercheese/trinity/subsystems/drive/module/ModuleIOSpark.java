package edu.msoe.cybercheese.trinity.subsystems.drive.module;

import static edu.msoe.cybercheese.trinity.subsystems.drive.DriveConstants.*;
import static edu.msoe.cybercheese.trinity.util.SparkUtil.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.msoe.cybercheese.trinity.odometry.OdometryCallback;
import edu.msoe.cybercheese.trinity.odometry.SparkSwerveModuleHardware;
import edu.msoe.cybercheese.trinity.util.hw.MotorIOSpark;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.function.DoubleSupplier;
import org.jspecify.annotations.Nullable;

public class ModuleIOSpark implements ModuleIO {
    private final ModuleDefinition moduleDef;

    private final Rotation2d zeroRotation;

    private final MotorIOSpark driveMotor;
    private final MotorIOSpark turnMotor;

    private final SparkSwerveModuleHardware odometryHal;

    public ModuleIOSpark(final ModuleDefinition moduleDef) {
        this.moduleDef = moduleDef;
        this.zeroRotation = moduleDef.zeroRotation();

        this.driveMotor = new MotorIOSpark(moduleDef.driveCanId(), DRIVE_MOTOR_CONFIG);
        this.turnMotor = new MotorIOSpark(moduleDef.turnCanId(), TURN_MOTOR_CONFIG);

        this.odometryHal = new SparkSwerveModuleHardware(this.driveMotor, this.turnMotor);

        tryUntilOk(this.driveMotor.spark(), 5, () -> this.driveMotor.encoder().setPosition(0.0));
    }

    @Override
    public void updateInputs(ModuleInputs inputs) {
        this.driveMotor.updateInputs(inputs.drive);
        this.turnMotor.updateInputs(inputs.turn);

        inputs.odometryTimestamps = this.odometryHal.timestamps.toDoubleArray();
        inputs.odometryDrivePositions = this.odometryHal.drivePositions.toDoubleArray();
        inputs.odometryTurnPositions = this.odometryHal.turnPositions.toDoubleArray();

        for (int i = 0; i < inputs.odometryTurnPositions.length; i++) {
            inputs.odometryTurnPositions[i] -= zeroRotation.getRadians();
        }
    }

    @Override
    public @Nullable OdometryCallback getOdometryCallback() {
        return this.odometryHal;
    }

    @Override
    public void setDriveOpenLoop(double output) {
        this.driveMotor.runOpenLoop(output);
    }

    @Override
    public void setTurnOpenLoop(double output) {
        this.turnMotor.runOpenLoop(output);
    }

    @Override
    public void setDriveVelocity(double velocityRadPerSec) {
        this.driveMotor.runVelocity(velocityRadPerSec);
    }

    @Override
    public void setTurnPosition(Rotation2d rotation) {
        this.turnMotor.runPosition(rotation.plus(zeroRotation).getRadians());
    }
}
