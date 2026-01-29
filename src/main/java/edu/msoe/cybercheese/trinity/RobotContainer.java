package edu.msoe.cybercheese.trinity;

import edu.msoe.cybercheese.trinity.commands.DriveCommands;
import edu.msoe.cybercheese.trinity.subsystems.drive.*;
import edu.msoe.cybercheese.trinity.subsystems.drive.gyro.GyroIO;
import edu.msoe.cybercheese.trinity.subsystems.drive.gyro.GyroIOPigeon2;
import edu.msoe.cybercheese.trinity.subsystems.drive.module.ModuleIO;
import edu.msoe.cybercheese.trinity.subsystems.drive.module.ModuleIOSim;
import edu.msoe.cybercheese.trinity.subsystems.drive.module.ModuleIOSpark;
import edu.msoe.cybercheese.trinity.subsystems.vision.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.ArrayList;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
    private final Drive drive;
    private final Vision vision;

    private final CommandXboxController controller = new CommandXboxController(0);

    private final LoggedDashboardChooser<Command> autoChooser;

    public RobotContainer() {
        this.drive = new Drive(
                this.createGyroIo(),
                this.createModuleIo(DriveConstants.MODULE_DEFINITIONS[0]),
                this.createModuleIo(DriveConstants.MODULE_DEFINITIONS[1]),
                this.createModuleIo(DriveConstants.MODULE_DEFINITIONS[2]),
                this.createModuleIo(DriveConstants.MODULE_DEFINITIONS[3]));

        final var cameras = new ArrayList<Camera>();
        for (final var cameraDef : VisionConstants.CAMERA_DEFINITIONS) {
            cameras.add(new Camera(cameraDef, this.createVisionIo(cameraDef)));
        }
        this.vision = new Vision(this.drive, cameras);

        this.autoChooser = new LoggedDashboardChooser<>("Auto Choices", new SendableChooser<>());
        this.setupSysIdAutoChooser();

        this.configureButtonBindings();
    }

    private void setupSysIdAutoChooser() {
        this.autoChooser.addOption(
                "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
        this.autoChooser.addOption(
                "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
        this.autoChooser.addOption(
                "Drive SysId (Quasistatic Forward)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        this.autoChooser.addOption(
                "Drive SysId (Quasistatic Reverse)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        this.autoChooser.addOption(
                "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        this.autoChooser.addOption(
                "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }

    private void configureButtonBindings() {
        // Default command, normal field-relative drive
        this.drive.setDefaultCommand(DriveCommands.joystickDrive(
                this.drive, () -> -controller.getLeftY(), () -> -controller.getLeftX(), () -> -controller.getRightX()));

        // Lock to 0 deg when A button is held
        this.controller
                .a()
                .whileTrue(DriveCommands.joystickDriveAtAngle(
                        this.drive,
                        () -> -controller.getLeftY(),
                        () -> -controller.getLeftX(),
                        () -> Rotation2d.kZero));

        this.controller.x().onTrue(Commands.runOnce(this.drive::stopWithX, this.drive));

        this.controller
                .b()
                .onTrue(Commands.runOnce(
                                () -> this.drive.setPose(
                                        new Pose2d(this.drive.getPose().getTranslation(), Rotation2d.kZero)),
                                this.drive)
                        .ignoringDisable(true));
    }

    public Command getAutonomousCommand() {
        return this.autoChooser.get();
    }

    private GyroIO createGyroIo() {
        // if (Constants.CURRENT_MODE == Constants.Mode.REAL) return new GyroIOCanandGyro();
        if (Constants.CURRENT_MODE == Constants.Mode.REAL) return new GyroIOPigeon2();

        return inputs -> {};
    }

    private ModuleIO createModuleIo(final DriveConstants.ModuleDefinition definition) {
        return switch (Constants.CURRENT_MODE) {
            case REAL -> new ModuleIOSpark(definition);
            case SIM -> new ModuleIOSim();
            case REPLAY -> inputs -> {};
        };
    }

    private VisionIO createVisionIo(final VisionConstants.CameraDefinition definition) {
        return switch (Constants.CURRENT_MODE) {
            case REAL -> new VisionIOPhotonVision(definition.name(), definition.transform());
            case SIM -> new VisionIOPhotonVisionSim(definition.name(), definition.transform(), this.drive::getPose);
            case REPLAY -> inputs -> {};
        };
    }
}
