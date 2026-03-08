package edu.msoe.cybercheese.trinity;

import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {
    public static final Mode SIM_MODE = Mode.SIM;
    public static final Mode CURRENT_MODE = RobotBase.isReal() ? Mode.REAL : SIM_MODE;

    public static final boolean ENABLE_SYSID = true;
    public static final boolean ENABLE_VISION_POSE_FUSION = true;

    public enum Mode {
        REAL,
        SIM,
        REPLAY
    }
}
