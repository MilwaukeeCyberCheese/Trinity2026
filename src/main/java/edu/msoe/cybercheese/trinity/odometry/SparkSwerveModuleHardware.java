package edu.msoe.cybercheese.trinity.odometry;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import edu.msoe.cybercheese.trinity.util.hw.MotorIOSpark;
import it.unimi.dsi.fastutil.doubles.DoubleArrayList;
import it.unimi.dsi.fastutil.doubles.DoubleList;

public class SparkSwerveModuleHardware implements OdometryCallback {

    private final MotorIOSpark driveMotor;
    private final MotorIOSpark turnMotor;

    public final DoubleList timestamps = new DoubleArrayList();
    public final DoubleList drivePositions = new DoubleArrayList();
    public final DoubleList turnPositions = new DoubleArrayList();

    public SparkSwerveModuleHardware(final MotorIOSpark driveMotor, final MotorIOSpark turnMotor) {
        this.driveMotor = driveMotor;
        this.turnMotor = turnMotor;
    }

    @Override
    public void clearFrame() {
        this.timestamps.clear();
        this.drivePositions.clear();
        this.turnPositions.clear();
    }

    @Override
    public void collectOdometry(double fpgaTime) {
        var isValid = true;

        final var drivePosition = this.driveMotor.encoder().getPosition();
        if (this.driveMotor.spark().getLastError() != REVLibError.kOk) {
            isValid = false;
        }

        final var turnPosition = this.turnMotor.absoluteEncoder().getPosition();
        if (this.turnMotor.spark().getLastError() != REVLibError.kOk) {
            isValid = false;
        }

        if (isValid) {
            this.timestamps.add(fpgaTime);
            this.drivePositions.add(drivePosition);
            this.turnPositions.add(turnPosition);
        }
    }
}
