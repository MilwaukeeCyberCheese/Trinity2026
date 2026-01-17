package edu.msoe.cybercheese.trinity.odometry;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;

public class SparkSwerveModuleHardware implements SwerveModuleHardware {

    private final SparkBase driveSpark;
    private final SparkBase turnSpark;
    private final RelativeEncoder driveEncoder;
    private final AbsoluteEncoder turnEncoder;

    public SparkSwerveModuleHardware(SparkBase driveSpark, SparkBase turnSpark, RelativeEncoder driveEncoder, AbsoluteEncoder turnEncoder) {
        this.driveSpark = driveSpark;
        this.turnSpark = turnSpark;
        this.driveEncoder = driveEncoder;
        this.turnEncoder = turnEncoder;
    }

    @Override
    public double readDrivePosition() {
        // TODO: error handling
        return this.driveEncoder.getPosition();
    }

    @Override
    public double readTurnPosition() {
        return this.turnEncoder.getPosition();
    }
}
