package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

public class Hopperfeeder extends SubsystemBase {
    private final SparkFlex rotary;
    private final RelativeEncoder rotaryEncoder;
    private final SparkMax feeder;
    private final RelativeEncoder feederEncoder;

  public Hopperfeeder() {
   // Change deviceId to 1 for testing 4 is a place holder before testing
    rotary = new SparkFlex(3, MotorType.kBrushless);
    rotaryEncoder = rotary.getEncoder();
  //Change deviceId to 2 for testing 5 is a place holder before testing
    feeder = new SparkMax(4, MotorType.kBrushless);
    feederEncoder = feeder.getEncoder();
  }


 //Sets speed for rotary
  public void setRotaryVelocity(double speed){
    rotary.set(speed);
    feeder.set(speed);
  }  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  
  }
}  