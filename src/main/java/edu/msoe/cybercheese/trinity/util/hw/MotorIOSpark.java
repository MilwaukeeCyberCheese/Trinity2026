package edu.msoe.cybercheese.trinity.util.hw;

import static edu.msoe.cybercheese.trinity.util.SparkUtil.*;
import static edu.msoe.cybercheese.trinity.util.SparkUtil.ifOk;
import static edu.msoe.cybercheese.trinity.util.SparkUtil.sparkStickyFault;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.msoe.cybercheese.trinity.replay.IO;
import edu.wpi.first.math.filter.Debouncer;
import java.util.function.DoubleSupplier;

public class MotorIOSpark implements IO<MotorInputs> {

    private final MotorConfig config;

    private final SparkBaseConfig sparkConfig;
    private final SparkBase spark;
    private final AbsoluteEncoder absoluteEncoder;
    private final SparkClosedLoopController controller;

    private final Debouncer connectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

    public MotorIOSpark(final MotorConfig config) {
        this.config = config;
        this.sparkConfig = config.sparkConfig();
        this.spark = config.kind() == MotorConfig.ControllerKind.FLEX
                ? new SparkFlex(config.canId(), SparkLowLevel.MotorType.kBrushless)
                : new SparkMax(config.canId(), SparkLowLevel.MotorType.kBrushless);
        this.absoluteEncoder = this.spark.getAbsoluteEncoder();
        this.controller = this.spark.getClosedLoopController();

        tryUntilOk(
                this.spark,
                5,
                () -> this.spark.configure(
                        this.sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    }

    public void periodic() {
        sparkStickyFault = false;
    }

    public void runOpenLoop(double output) {
        this.spark.setVoltage(output);
    }

    // TODO: doc that this is rads/sec
    public void runVelocity(double velocity) {}

    public void runPosition(double position) {}

    @Override
    public void updateInputs(MotorInputs inputs) {
        ifOk(this.spark, this.absoluteEncoder::getPosition, (value) -> inputs.position = value);
        ifOk(this.spark, this.absoluteEncoder::getVelocity, (value) -> inputs.velocity = value);
        ifOk(
                this.spark,
                new DoubleSupplier[] {this.spark::getAppliedOutput, this.spark::getBusVoltage},
                (values) -> inputs.appliedVolts = values[0] * values[1]);
        ifOk(this.spark, this.spark::getOutputCurrent, (value) -> inputs.currentAmps = value);
        inputs.connected = this.connectedDebounce.calculate(!sparkStickyFault);
    }
}
