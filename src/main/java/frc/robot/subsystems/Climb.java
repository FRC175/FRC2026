// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;


public class Climb extends SubsystemBase {
  
  private final SparkMax climbMotor;
  private final RelativeEncoder climbEncoder;

  public Climb() {
    climbMotor = new SparkMax(1, MotorType.kBrushless);
    
    climbEncoder = climbMotor.getEncoder();
  }

 
/**
 * Sets the speed of the climb motor.
 * @param speed Speed to set the climb motor (-1.0 to 1.0)
 */
public void setSpeed(double speed) {
    climbMotor.set(speed);
}

  /**
   * Method to find and return the climb motor speed.
   *
   * @return climb  motor speed.
   */
  public double getMotorRPM() {
    double motorSpeed = climbEncoder.getVelocity();
    return motorSpeed;
  }
/**
 * A statement that sets the climb speed that is set positive if up is true and false if up is false. 
 * @param up True is running the climb up, false if running the climb down.
 * @param speed The speed the climb motor is set to (-1.0 to 1.0). 
 */
  public void climbSpeed(boolean up,double speed) {
    if(up) setSpeed(speed);
    else setSpeed(-speed);
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
