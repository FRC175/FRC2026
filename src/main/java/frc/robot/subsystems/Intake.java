package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

public class Intake extends SubsystemBase {
    private final SparkMax intake;
    private final RelativeEncoder intakeEncoder;
 
    public Intake() {
   //Change deviceId to 1 for testing 6 is a place holder before testing
    intake = new SparkMax(2, MotorType.kBrushless);
    intakeEncoder = intake.getEncoder();
  }
public void setIntakeVelocity(double speed) {
    intake.set(speed);
  }
public double getEncoderValue(){
        double leaderDeployReading = intakeEncoder.getVelocity();
        return leaderDeployReading; 
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
