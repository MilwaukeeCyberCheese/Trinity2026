package edu.msoe.cybercheese.trinity.util.hw;

import static edu.msoe.cybercheese.trinity.util.SparkUtil.*;
import static edu.msoe.cybercheese.trinity.util.SparkUtil.ifOk;
import static edu.msoe.cybercheese.trinity.util.SparkUtil.sparkStickyFault;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import java.util.function.DoubleSupplier;

public class MotorIOSpark implements MotorIO {

    protected final MotorConfig config;

    protected final SparkBaseConfig sparkConfig;
    protected final SparkBase spark;
    protected final RelativeEncoder encoder;
    protected final AbsoluteEncoder absoluteEncoder;
    protected final SparkClosedLoopController controller;

    protected final Debouncer connectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

    public MotorIOSpark(final int canId, final MotorConfig config) {
        this.config = config;
        this.sparkConfig = config.sparkConfig();
        this.spark = config.kind() == MotorConfig.ControllerKind.FLEX
                ? new SparkFlex(canId, SparkLowLevel.MotorType.kBrushless)
                : new SparkMax(canId, SparkLowLevel.MotorType.kBrushless);
        this.encoder = this.spark.getEncoder();
        this.absoluteEncoder = this.spark.getAbsoluteEncoder();
        this.controller = this.spark.getClosedLoopController();

        tryUntilOk(
                this.spark,
                5,
                () -> this.spark.configure(
                        this.sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    }

    public SparkBase spark() {
        return this.spark;
    }

    public RelativeEncoder encoder() {
        return this.encoder;
    }

    public AbsoluteEncoder absoluteEncoder() {
        return this.absoluteEncoder;
    }

    public SparkClosedLoopController controller() {
        return this.controller;
    }

    public void runOpenLoop(double output) {
        this.spark.setVoltage(output);
    }

    @Override
    public void runVelocity(double velocity) {
        final var ff = this.config.feedForward().ks() * Math.signum(velocity)
                + this.config.feedForward().kv() * velocity;

        this.controller.setSetpoint(
                velocity,
                SparkBase.ControlType.kVelocity,
                ClosedLoopSlot.kSlot0,
                ff,
                SparkClosedLoopController.ArbFFUnits.kVoltage);
    }

    @Override
    public void runPosition(double position) {
        final var setpoint = MathUtil.inputModulus(position, 0, 2. * Math.PI);

        this.controller.setSetpoint(setpoint, SparkBase.ControlType.kPosition);
    }

    @Override
    public void updateInputs(MotorIO.MotorInputs inputs) {
        sparkStickyFault = false;

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
