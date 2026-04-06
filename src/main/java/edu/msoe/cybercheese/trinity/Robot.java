package edu.msoe.cybercheese.trinity;

import com.revrobotics.util.StatusLogger;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.jspecify.annotations.Nullable;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
    private static final int PDH_CAN_ID = 1;
    private static final String GAMING_MODE_DASHBOARD_KEY = "GamingModeActive";

    private final RobotContainer robotContainer;
    private final @Nullable PowerDistribution powerDistribution;

    private @Nullable Command autonomousCommand;

    public Robot() {
        DriverStation.silenceJoystickConnectionWarning(true);
        // java.util.logging.Logger.getLogger("edu.wpi.first.wpilibj.Notifier").setLevel(Level.SEVERE);
        // java.util.logging.Logger.getLogger("edu.wpi.first.wpilibj.RobotBase").setLevel(Level.SEVERE);

        Logger.recordMetadata("BuildTimestamp", BuildConstants.BUILD_TIME.toString());
        Logger.recordMetadata("ImplementationTitle", BuildConstants.TITLE);
        Logger.recordMetadata("ImplementationVersion", BuildConstants.VERSION);

        Logger.recordMetadata("GitCommit", BuildConstants.GIT_COMMIT);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);

        Logger.recordMetadata("Mode", Constants.CURRENT_MODE.toString());

        switch (Constants.CURRENT_MODE) {
            case REAL -> {
                Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());
            }
            case SIM -> Logger.addDataReceiver(new NT4Publisher());
            case REPLAY -> {
                this.setUseTiming(false);
                final var logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
            }
        }

        Logger.registerURCL(URCL.startExternal());
        StatusLogger.disableAutoLogging();

        // WPILib only publishes PDH telemetry after the device is instantiated.
        this.powerDistribution = Constants.CURRENT_MODE == Constants.Mode.REPLAY
                ? null
                : new PowerDistribution(PDH_CAN_ID, ModuleType.kRev);

        Logger.start();

        this.robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        // Optionally switch the thread to high priority to improve loop
        // timing (see the template project documentation for details)
        // Threads.setCurrentThreadPriority(true, 99);

        CommandScheduler.getInstance().run();
        this.robotContainer.logAllianceDiagnostics();

        // Return to non-RT thread priority (do not modify the first argument)
        // Threads.setCurrentThreadPriority(false, 10);

        SmartDashboard.updateValues();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void autonomousInit() {
        this.autonomousCommand = this.robotContainer.getAutonomousCommand();
        CommandScheduler.getInstance().schedule(this.autonomousCommand);
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        if (this.autonomousCommand != null) {
            this.autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {
        SmartDashboard.putBoolean(GAMING_MODE_DASHBOARD_KEY, true);
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void simulationInit() {}

    @Override
    public void simulationPeriodic() {
        this.robotContainer.updateSimulation();
    }
}
