package edu.msoe.cybercheese.trinity.subsystems.vision;

import edu.msoe.cybercheese.trinity.replay.LogUtil;
import edu.wpi.first.wpilibj.Alert;

public record Camera(
    VisionConstants.CameraDefinition definition,
    VisionIO io,
    VisionIO.VisionInputs inputs,
    Alert disconnectedAlert) {
  public Camera(VisionConstants.CameraDefinition definition, VisionIO io) {
    this(
        definition,
        io,
        new VisionIO.VisionInputs(),
        new Alert(
            "Vision camera " + definition.name() + " is disconnected.", Alert.AlertType.kWarning));
  }

  public void update() {
    LogUtil.update("Vision/Camera/" + this.definition.name(), this.io, this.inputs);

    this.disconnectedAlert.set(!this.inputs.connected);
  }
}
