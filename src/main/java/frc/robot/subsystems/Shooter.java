// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;

import frc.robot.subsystems.Limelight;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  private static Shooter instance;
  private final SparkFlex shooterLeader;
  private final SparkFlex shooterFollower;
  private final RelativeEncoder leaderEncoder, followerEncoder;
  private final Servo leftServoHood;
  private final Servo rightServoHood;
  public final PIDController velocityController;

  public boolean closeEnough;

  public Boolean shooterRunning;
  private double flywheelEffort;

  /** 
   * Creates a new Shooter Subsystem 
   * */
  public Shooter() {
    shooterLeader = new SparkFlex(ShooterConstants.shooterLeaderID, MotorType.kBrushless);
    shooterFollower = new SparkFlex(ShooterConstants.shooterFollowID, MotorType.kBrushless);

    leaderEncoder = shooterLeader.getEncoder();
    followerEncoder = shooterFollower.getEncoder();

    leftServoHood = new Servo(ShooterConstants.leftHoodServo);
    leftServoHood.setBoundsMicroseconds(2000, 1500, 1500, 1500, 1000);
    rightServoHood = new Servo(ShooterConstants.rightHoodServo);
    rightServoHood.setBoundsMicroseconds(2000, 1500, 1500, 1500, 1000);

    shooterRunning = false;
    flywheelEffort = 0;
    velocityController = new PIDController(.000000375, 0.00001, 0);
    velocityController.setSetpoint(ShooterConstants.baseVelocity);
    velocityController.setTolerance(50);
    velocityController.setIZone(500);
    closeEnough = false;

  }
  
  /**
   * Returns the initialized shooter subsystem, or creates a shooter if there is not one already
   * @return The current shooter instance
   */
  public static Shooter getInstance() {
    if(instance == null) {
      instance = new Shooter();
    }
    return instance;
  }

  /**
   * Sets to shooting state and sets motor to effort calculated by PID
   */
  public void run(){
    this.shooterRunning = true;
    shooterLeader.set(-flywheelEffort);
  }

  /**
   * Stops the shooter motors
   */
  public void stop() {
    this.shooterRunning = false;
    shooterLeader.set(0);
  }

  /**
   * Gets the average velocity of encoders
   * @return avererage velocity(rpm)
   */
  public double getVelocity() {
    return leaderEncoder.getVelocity();
  }

  /**
   * Returns the pose of the hood-servo
   * @return The hood-servo pose
   */
  public double getServoPose() {
    return leftServoHood.get();
  }

  /**
   * Configures the motor controllers for the shooter.
   * We use default configuration other than the one motor being a follower, but more can be added
   */
  public void configureFlexes() {
    SparkFlexConfig followConfig = new SparkFlexConfig();
    followConfig.follow(ShooterConstants.shooterLeaderID, true);
    
    shooterFollower.configure(followConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkFlexConfig leaderConfig = new SparkFlexConfig();
    shooterLeader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

/**
 * Checks if flywheel speed is at the goal speed. 
 * @param goalSpeed The intended speed of the flywheel
 * @return true if goal speed is greater thn or equal to flywheel speed, else false
 */
  public boolean flywheelAtSpeed(double goalSpeed) {
    if (Math.abs(getVelocity()) >= Math.abs(goalSpeed)) {
      return true;
    } else
      return false;
  }

/**
 * Getting the angle of the shooter hood, as the average of the two controlling servo positiongs
 * @return returning the servo hood position
 */
  public double getHoodPosition() {
    double left = leftServoHood.get();
    double right = rightServoHood.get();
    return (left + right) / 2;
  }

/**
 * setting the servo hood value
 * @param value returning the servo hood value
 */
  public void setServoHood(double value) {
    leftServoHood.set(value);
    rightServoHood.set(value);
  }

  public double calculate(Limelight limelight) {
    double hoodPosition = 0;
    double distance = limelight.getZ();
    hoodPosition = .59 - (.565 * distance) + (.151 * (distance * distance));
    return hoodPosition;
    
  }

  @Override
  public void periodic() {
    
  
    SmartDashboard.putNumber("servoPose", getServoPose());
    if(shooterRunning) {
      flywheelEffort = velocityController.calculate(getVelocity(), ShooterConstants.baseVelocity);
      //flywheelEffort *= ShooterConstants.baseEffort;
      double error = velocityController.getError();
      flywheelEffort += .000075 * error;
      flywheelEffort = MathUtil.clamp(flywheelEffort, -1, 1);
    } else {
      flywheelEffort = 0;
    }
    SmartDashboard.putNumber("Flywheel effort", flywheelEffort);
    SmartDashboard.putNumber("Flywheel Velocity", (getVelocity()));
    SmartDashboard.putBoolean("Close Enough?", closeEnough);
    shooterLeader.set(-flywheelEffort);

  }
}
