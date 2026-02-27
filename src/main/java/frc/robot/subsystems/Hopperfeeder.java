package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

public class Hopperfeeder extends SubsystemBase {
    private final SparkFlex rotary, feederWheel;
    private final RelativeEncoder rotaryEncoder, feederTrackEncoder, feederWheelEncoder;
    private final SparkMax feederTrack;

  public Hopperfeeder() {
    rotary = new SparkFlex(15, MotorType.kBrushless);
    rotaryEncoder = rotary.getEncoder();
  
    feederTrack = new SparkMax(14, MotorType.kBrushless);
    feederTrackEncoder = feederTrack.getEncoder();

    feederWheel = new SparkFlex(16, MotorType.kBrushless);
    feederWheelEncoder = feederWheel.getEncoder();
  }


 //Sets speed for rotary
  public void setHopperVelocity(double speed){
    SmartDashboard.putString("setting?", "YEP :)");
    rotary.set(speed);
    feederTrack.set(speed);
    feederWheel.set(speed);
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