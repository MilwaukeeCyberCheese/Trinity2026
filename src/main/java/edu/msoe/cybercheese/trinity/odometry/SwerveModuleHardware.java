package edu.msoe.cybercheese.trinity.odometry;

interface SwerveModuleHardware {

    boolean isConnected();
    double readDrivePosition();
    double readTurnPosition();
}
