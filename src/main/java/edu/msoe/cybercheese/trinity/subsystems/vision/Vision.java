package edu.msoe.cybercheese.trinity.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
    private final VisionConsumer consumer;
    private final List<Camera> cameras;

    public Vision(final VisionConsumer consumer, final List<Camera> cameras) {
        this.consumer = consumer;
        this.cameras = cameras;
    }

    public Rotation2d getTargetX(int cameraIndex) {
        return this.cameras.get(cameraIndex).inputs().latestTargetObservation.tx();
    }

    @Override
    public void periodic() {
        final var allTagPoses = new ArrayList<Pose3d>();
        final var allRobotPoses = new ArrayList<Pose3d>();
        final var allRobotPosesAccepted = new ArrayList<Pose3d>();
        final var allRobotPosesRejected = new ArrayList<Pose3d>();

        for (final var camera : this.cameras) {
            camera.update();

            final var tagPoses = new ArrayList<Pose3d>();
            final var robotPoses = new ArrayList<Pose3d>();
            final var robotPosesAccepted = new ArrayList<Pose3d>();
            final var robotPosesRejected = new ArrayList<Pose3d>();

            for (final var tag : camera.inputs().tagIds) {
                final var pose = VisionConstants.APRIL_TAG_LAYOUT.getTagPose(tag);
                pose.ifPresent(tagPoses::add);
            }

            for (final var observation : camera.inputs().poseObservations) {
                robotPoses.add(observation.pose());

                if (observation.shouldRejectPose()) {
                    robotPosesRejected.add(observation.pose());
                    continue;
                }
                robotPosesAccepted.add(observation.pose());

                this.consumer.addVisionMeasurement(
                        observation.pose().toPose2d(),
                        observation.timestamp(),
                        observation
                                .computeStandardDeviations()
                                .times(camera.definition().stdDevFactor()));
            }

            Logger.recordOutput(
                    "Vision/Camera/" + camera.definition().name() + "/TagPoses", tagPoses.toArray(new Pose3d[0]));
            Logger.recordOutput(
                    "Vision/Camera/" + camera.definition().name() + "/RobotPoses", robotPoses.toArray(new Pose3d[0]));
            Logger.recordOutput(
                    "Vision/Camera/" + camera.definition().name() + "/RobotPosesAccepted",
                    robotPosesAccepted.toArray(new Pose3d[0]));
            Logger.recordOutput(
                    "Vision/Camera/" + camera.definition().name() + "/RobotPosesRejected",
                    robotPosesRejected.toArray(new Pose3d[0]));
            allTagPoses.addAll(tagPoses);
            allRobotPoses.addAll(robotPoses);
            allRobotPosesAccepted.addAll(robotPosesAccepted);
            allRobotPosesRejected.addAll(robotPosesRejected);
        }

        // Log summary data
        Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[0]));
        Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[0]));
        Logger.recordOutput("Vision/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(new Pose3d[0]));
        Logger.recordOutput("Vision/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(new Pose3d[0]));
    }

    @FunctionalInterface
    public interface VisionConsumer {
        void addVisionMeasurement(
                Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs);
    }
}
