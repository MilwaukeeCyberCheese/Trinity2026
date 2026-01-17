package edu.msoe.cybercheese.trinity.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;

import org.photonvision.PhotonCamera;

public class VisionIOPhotonVision implements VisionIO {
    protected final PhotonCamera camera;
    protected final Transform3d robotToCamera;

    public VisionIOPhotonVision(String name, Transform3d robotToCamera) {
        this.camera = new PhotonCamera(name);
        this.robotToCamera = robotToCamera;
    }

    @Override
    public void updateInputs(VisionInputs inputs) {
        inputs.connected = this.camera.isConnected();

        inputs.tagIds.clear();
        inputs.poseObservations.clear();

        for (var result : this.camera.getAllUnreadResults()) {
            if (result.hasTargets()) {
                inputs.latestTargetObservation = new TargetObservation(
                        Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
                        Rotation2d.fromDegrees(result.getBestTarget().getPitch())
                );
            } else {
                inputs.latestTargetObservation = new TargetObservation(Rotation2d.kZero, Rotation2d.kZero);
            }

            if (result.multitagResult.isPresent()) {
                var multitagResult = result.multitagResult.get();

                Transform3d fieldToCamera = multitagResult.estimatedPose.best;
                Transform3d fieldToRobot = fieldToCamera.plus(this.robotToCamera.inverse());
                Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

                double totalTagDistance = 0.0;
                for (var target : result.targets) {
                    totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
                }

                inputs.tagIds.addAll(multitagResult.fiducialIDsUsed);

                inputs.poseObservations.add(new PoseObservation(
                        result.getTimestampSeconds(),
                        robotPose,
                        multitagResult.estimatedPose.ambiguity,
                        multitagResult.fiducialIDsUsed.size(),
                        totalTagDistance / result.targets.size(),
                        PoseObservationType.PHOTONVISION
                ));

            } else if (!result.targets.isEmpty()) {
                var target = result.targets.get(0);

                var tagPose = VisionConstants.APRIL_TAG_LAYOUT.getTagPose(target.fiducialId);
                if (tagPose.isPresent()) {
                    Transform3d fieldToTarget = new Transform3d(tagPose.get().getTranslation(), tagPose.get().getRotation());
                    Transform3d cameraToTarget = target.bestCameraToTarget;
                    Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
                    Transform3d fieldToRobot = fieldToCamera.plus(this.robotToCamera.inverse());
                    Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

                    inputs.tagIds.add((short) target.fiducialId);

                    inputs.poseObservations.add(new PoseObservation(
                            result.getTimestampSeconds(),
                            robotPose,
                            target.poseAmbiguity,
                            1,
                            cameraToTarget.getTranslation().getNorm(),
                            PoseObservationType.PHOTONVISION
                    ));
                }
            }
        }
    }
}
