package edu.msoe.cybercheese.trinity;

import com.revrobotics.util.StatusLogger;
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
    private final RobotContainer robotContainer;

    private @Nullable Command autonomousCommand;

    public Robot() {
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

        Logger.start();

        this.robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        // Optionally switch the thread to high priority to improve loop
        // timing (see the template project documentation for details)
        // Threads.setCurrentThreadPriority(true, 99);

        CommandScheduler.getInstance().run();

        // Return to non-RT thread priority (do not modify the first argument)
        // Threads.setCurrentThreadPriority(false, 10);
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
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void simulationInit() {}

    @Override
    public void simulationPeriodic() {}
}
