package edu.msoe.cybercheese.trinity.subsystems.shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.spark.SparkMax;


public class Shooter extends SubsystemBase{
    private SparkMax m_motor1 = new SparkMax(3, MotorType.kBrushless);
    private Double velocity = 0.0;
     
    
    public Command shootCommand(){
        return this.runOnce(() -> m_motor1.set(this.velocity));
    }
}
