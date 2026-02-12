// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.Servo;
public class Shooter extends SubsystemBase {
    private final SparkFlex shooterLeader;
    //private final SparkFlex shooterFollower;
    private final RelativeEncoder leaderEncoder;
    private final Servo servoHood;
    //private final RelativeEncoder followerEncoder;
  /** Creates a new ExampleSubsystem. */
  public Shooter() {
    shooterLeader = new SparkFlex(10, MotorType.kBrushless);
    //shooterFollower = new SparkFlex(3, MotorType.kBrushless);
    leaderEncoder = shooterLeader.getEncoder();
    //followerEncoder = shooterFollower.getEncoder()
    servoHood = new Servo(9);
  }
  
  /**
   * Sets the shooter motors to given speed (follower motor opposite direction of leader)
   * @param speed Speed to set shooter motors (-1 to 1)
   */
  public void setShooterVelocity(double speed){
    shooterLeader.set(speed);
    //shooterFollower.set(-speed);
  }
  /**
   * Gets the average velocity of encoders
   * @return avererage velocity(rpm)
   */ 
  public double getEncoderValue(){
        double leaderReading = leaderEncoder.getVelocity();
        //double followerReading = followerEncoder.getVelocity();
        //followerReading *= -1;
        return leaderReading; //(leaderReading + followerReading)/2;
    } 
  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }
  public double getPosition(){
    double answer = servoHood.get();
    return answer;
  }
  public void setServoHood(double value){
    servoHood.set(value);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
