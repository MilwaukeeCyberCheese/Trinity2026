package edu.msoe.cybercheese.trinity.replay;

import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface IO<T extends LoggableInputs> {

  void updateInputs(T inputs);
}
