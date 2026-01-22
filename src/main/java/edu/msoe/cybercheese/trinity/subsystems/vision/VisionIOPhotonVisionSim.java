package edu.msoe.cybercheese.trinity.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class VisionIOPhotonVisionSim extends VisionIOPhotonVision {
  private static final VisionSystemSim VISION_SIM = new VisionSystemSim("main");

  static {
    VISION_SIM.addAprilTags(VisionConstants.APRIL_TAG_LAYOUT);
  }

  private final Supplier<Pose2d> poseSupplier;
  private final PhotonCameraSim cameraSim;

  public VisionIOPhotonVisionSim(
      String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {
    super(name, robotToCamera);
    this.poseSupplier = poseSupplier;

    var cameraProperties = new SimCameraProperties();
    this.cameraSim =
        new PhotonCameraSim(camera, cameraProperties, VisionConstants.APRIL_TAG_LAYOUT);
    VISION_SIM.addCamera(this.cameraSim, robotToCamera);
  }

  @Override
  public void updateInputs(VisionInputs inputs) {
    VISION_SIM.update(poseSupplier.get());
    super.updateInputs(inputs);
  }
}
