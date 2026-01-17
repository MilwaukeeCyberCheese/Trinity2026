package edu.msoe.cybercheese.trinity.odometry;

interface GyroHardware {

    boolean isConnected();

    double readYaw();
}
