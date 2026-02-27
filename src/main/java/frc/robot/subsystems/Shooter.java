// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkParameters;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends SubsystemBase {
  private final SparkFlex shooterLeader;
   private final SparkFlex shooterFollower;
  private final RelativeEncoder leaderEncoder, followerEncoder;
  private final Servo servoHood;

  // private final RelativeEncoder followerEncoder;
  /** Creates a new ExampleSubsystem. */
  public Shooter() {
    shooterLeader = new SparkFlex(11, MotorType.kBrushless);
    shooterFollower = new SparkFlex(12, MotorType.kBrushless);
    leaderEncoder = shooterLeader.getEncoder();
    followerEncoder = shooterFollower.getEncoder();
    servoHood = new Servo(0);

    configureFlexes();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("leader", shooterLeader.getAppliedOutput());
    SmartDashboard.putNumber("follower", shooterFollower.getAppliedOutput());
  }



  /**
   * Sets the shooter motors to given speed (follower motor opposite direction of
   * leader)
   * 
   * @param speed Speed to set shooter motors (-1 to 1)
   */
  public void setShooterVelocity(double speed) {
    shooterLeader.set(speed);
    //shooterFollower.set(speed);
  }

  /**
   * Stops the shooter motors
   */
  public void stopShooter() {
    shooterLeader.set(0);
    //shooterFollower.set(0);
  }

  /**
   * Gets the average velocity of encoders
   * 
   * @return avererage velocity(rpm)
   */
  public double getEncoderValue() {
    double leaderReading = leaderEncoder.getVelocity();
    double followerReading = followerEncoder.getVelocity();
    leaderReading *= -1;
    return Math.abs((leaderReading + followerReading)/2);
  }
  /**
   * Sets the servo position and the shooter speed
   * @param speed speed of the shooter
   * @param angle position of the hood/servo
   */
  public void setShooter(double speed, double angle) {
    double servoPose = angle; // with conversion
    setServoHood(servoPose);
    setShooterVelocity(speed);
  }
  public double getServoPose() {
    return servoHood.getAngle();
  }

  public void configureFlexes() {
    SparkFlexConfig followConfig = new SparkFlexConfig();
    followConfig.follow(11, true);
    
    shooterFollower.configure(followConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
/**
 * Checks if flywheel speed is at the goal speed. 
 * @param goalSpeed The intended speed of the flywheel
 * @return true if goal speed is greater thn or equal to flywheel speed, else false
 */
  public boolean flywheelAtSpeed(double goalSpeed) {
    if (getEncoderValue() >= goalSpeed) {
      return true;
    } else
      return false;
  }
/**
 * Getting the servo hood position
 * @return returning the servo hood position
 */
  public double getPosition() {
    double answer = servoHood.get();
    return answer;
  }
/**
 * setting the servo hood value
 * @param value returning the servo hood value
 */
  public void setServoHood(double value) {
    servoHood.set(value);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
