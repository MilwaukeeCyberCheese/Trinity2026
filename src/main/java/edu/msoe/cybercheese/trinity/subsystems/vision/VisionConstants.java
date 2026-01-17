package edu.msoe.cybercheese.trinity.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

import java.util.List;

public class VisionConstants {
    public record CameraDefinition(
            String name,
            Transform3d transform,
            double stdDevFactor
    ) {}

    public static final AprilTagFieldLayout APRIL_TAG_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    public static final List<CameraDefinition> CAMERA_DEFINITIONS = List.of(
            new CameraDefinition(
                    "camera_0",
                    new Transform3d(0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, 0.0)),
                    1.0
            ),
            new CameraDefinition(
                    "camera_1",
                    new Transform3d(-0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, Math.PI)),
                    1.0
            )
    );

    public static final double MAX_AMBIGUITY = 0.3;
    // Maximum absolute difference in height from ground
    public static final double MAX_Z_ERROR = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static final double LINEAR_STD_DEV_BASELINE = 0.02; // Meters
    public static final double ANGULAR_STD_DEV_BASELINE = 0.06; // Radians
}
