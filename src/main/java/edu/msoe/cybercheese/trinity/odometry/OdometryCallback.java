package edu.msoe.cybercheese.trinity.odometry;

public interface OdometryCallback {

    void clearFrame();

    void collectOdometry(double fpgaTime);
}
