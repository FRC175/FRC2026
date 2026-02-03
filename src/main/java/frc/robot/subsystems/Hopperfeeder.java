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
    private final SparkFlex spinner;
    private final RelativeEncoder spinnerEncoder;


  public Hopperfeeder() {
   //Change deviceId to 2 for testing 4 is a place holder before testing
    spinner = new SparkFlex(4, MotorType.kBrushless);
    spinnerEncoder = spinner.getEncoder();
  }


 //Sets speed for spinner
  public void setSpinerVelocity(double speed){
    spinner.set(speed);
  }
}

 public double getEncoderValue(){
        double spinnerReading = spinnerEncoder.getVelocity();
        return spinnerReading; 
    
 public class shooterFeeder extends SubsystemBase{
        private final SparkMax feeder;
        private final RelativeEncoder feederEncoder;
    }
 public shooterFeeder() {
     = new SparkMax(5, MotorType.kBrushless)
 }
    
}
    
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  
}

