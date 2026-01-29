package edu.msoe.cybercheese.trinity.odometry;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import it.unimi.dsi.fastutil.doubles.DoubleArrayList;
import it.unimi.dsi.fastutil.doubles.DoubleList;

public class SparkSwerveModuleHardware implements OdometryCallback {

    private final SparkBase driveSpark;
    private final SparkBase turnSpark;
    private final RelativeEncoder driveEncoder;
    private final AbsoluteEncoder turnEncoder;

    public final DoubleList timestamps = new DoubleArrayList();
    public final DoubleList drivePositions = new DoubleArrayList();
    public final DoubleList turnPositions = new DoubleArrayList();

    public SparkSwerveModuleHardware(
            SparkBase driveSpark, SparkBase turnSpark, RelativeEncoder driveEncoder, AbsoluteEncoder turnEncoder) {
        this.driveSpark = driveSpark;
        this.turnSpark = turnSpark;
        this.driveEncoder = driveEncoder;
        this.turnEncoder = turnEncoder;
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

        final var drivePosition = this.driveEncoder.getPosition();
        if (this.driveSpark.getLastError() != REVLibError.kOk) {
            isValid = false;
        }

        final var turnPosition = this.turnEncoder.getPosition();
        if (this.turnSpark.getLastError() != REVLibError.kOk) {
            isValid = false;
        }

        if (isValid) {
            this.timestamps.add(fpgaTime);
            this.drivePositions.add(drivePosition);
            this.turnPositions.add(turnPosition);
        }
    }
}
