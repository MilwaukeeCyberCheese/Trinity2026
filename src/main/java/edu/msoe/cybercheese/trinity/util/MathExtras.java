package edu.msoe.cybercheese.trinity.util;

import edu.wpi.first.wpilibj.DriverStation;

public class MathExtras {

    public static boolean isFlipped() {
        return DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    }
}
