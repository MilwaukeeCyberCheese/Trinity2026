package edu.msoe.cybercheese.trinity.util.hw;

import static edu.msoe.cybercheese.trinity.util.SparkUtil.*;
import static edu.msoe.cybercheese.trinity.util.SparkUtil.ifOk;
import static edu.msoe.cybercheese.trinity.util.SparkUtil.sparkStickyFault;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import java.util.function.DoubleSupplier;

public class Motor {

    private final MotorConfig config;

    private final MotorInputs inputs = new MotorInputs();

    private final SparkBaseConfig sparkConfig;
    private final SparkBase spark;

    public Motor(final MotorConfig config) {
        this.config = config;
        this.sparkConfig = config.sparkConfig();
        this.spark = config.isFlex()
                ? new SparkFlex(config.canId(), SparkLowLevel.MotorType.kBrushless)
                : new SparkMax(config.canId(), SparkLowLevel.MotorType.kBrushless);

        tryUntilOk(
                this.spark,
                5,
                () -> this.spark.configure(
                        this.sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    }

    public void periodic() {
        sparkStickyFault = false;
//        ifOk(spark, encoder::getPosition, (value) -> inputs.position = value);
//        ifOk(spark, encoder::getVelocity, (value) -> inputs.velocity = value);
//        ifOk(
//                spark,
//                new DoubleSupplier[] {spark::getAppliedOutput, spark::getBusVoltage},
//                (values) -> inputs.appliedVolts = values[0] * values[1]);
//        ifOk(spark, spark::getOutputCurrent, (value) -> inputs.currentAmps = value);
//        inputs.connected = connectedDebounce.calculate(!sparkStickyFault);
    }
}
