package edu.msoe.cybercheese.trinity.subsystems.vision;

import edu.msoe.cybercheese.trinity.replay.IO;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface VisionIO extends IO<VisionIO.VisionInputs> {

  @AutoLog
  class VisionInputs implements LoggableInputs {
    public boolean connected = false;
    public TargetObservation latestTargetObservation =
        new TargetObservation(Rotation2d.kZero, Rotation2d.kZero);
    public List<PoseObservation> poseObservations = new ArrayList<>();
    public Set<Short> tagIds = new HashSet<>();
  }

  record TargetObservation(Rotation2d tx, Rotation2d ty) {}

  record PoseObservation(
      double timestamp,
      Pose3d pose,
      double ambiguity,
      int tagCount,
      double averageTagDistance,
      PoseObservationType type) {
    public boolean shouldRejectPose() {
      if (this.tagCount() == 0) return true;

      if (this.tagCount() == 1 && this.ambiguity() > VisionConstants.MAX_AMBIGUITY) return true;

      if (Math.abs(this.pose().getZ()) > VisionConstants.MAX_Z_ERROR) return true;

      if (this.pose().getX() < 0.0
          || this.pose().getX() > VisionConstants.APRIL_TAG_LAYOUT.getFieldLength()
          || this.pose().getY() < 0.0
          || this.pose().getY() > VisionConstants.APRIL_TAG_LAYOUT.getFieldWidth()) return true;

      return false;
    }

    public Matrix<N3, N1> computeStandardDeviations() {
      double stdDevFactor = Math.pow(this.averageTagDistance(), 2.0) / this.tagCount();
      double linearStdDev = VisionConstants.LINEAR_STD_DEV_BASELINE * stdDevFactor;
      double angularStdDev = VisionConstants.ANGULAR_STD_DEV_BASELINE * stdDevFactor;

      return VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev);
    }
  }

  enum PoseObservationType {
    PHOTONVISION
  }
}
