// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;

import frc.robot.subsystems.Limelight;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  private static Shooter instance;
  private final SparkFlex shooterLeader;
  private final SparkFlex shooterFollower;
  private final SparkMax hoodMotor;
  private final RelativeEncoder leaderEncoder, followerEncoder;

  public final PIDController velocityController, hoodController;
  private final SimpleMotorFeedforward feedForeward;
  public double currentSetpoint, shooterHeading;

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

    // leftServoHood = new Servo(ShooterConstants.leftHoodServo);
    // leftServoHood.setBoundsMicroseconds(2000, 1500, 1500, 1500, 1000);
    // rightServoHood = new Servo(ShooterConstants.rightHoodServo);
    // rightServoHood.setBoundsMicroseconds(2000, 1500, 1500, 1500, 1000);

    hoodMotor = new SparkMax(ShooterConstants.hoodMotorID, MotorType.kBrushless);
    //rightHoodMotor = new SparkMax(0, MotorType.kBrushless);
    

    shooterRunning = false;
    flywheelEffort = 0;
    velocityController = new PIDController(.00000000045, 0.000016, 0);
    velocityController.setSetpoint(ShooterConstants.baseVelocity);
    velocityController.setTolerance(25);
    velocityController.setIZone(500);

    feedForeward = new SimpleMotorFeedforward( 0, 0.000001 );
    
    currentSetpoint = ShooterConstants.FrontHubSpeed;

    hoodController = new PIDController(.2, 0, 0);
    
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
   * Updates the running parameter measuring the shooter's heading (in reference to field positive x axis)
   * @param robotHeading The robot's heading in radians
   */
  public void updateHeading(double robotHeading) {
    shooterHeading = robotHeading - (Math.PI / 2);
  }
  

  public void setHoodNeo(double setpoint){
    if (!(getHoodPose() >= setpoint)){
      setHoodVelocity(.05);
    } else {
      setHoodVelocity(0);
    }
  }

  /**
   * Returns the shooter's heading (in reference to field positive x axis)
   * @return Shooter's heading in radians
   */
  public double getHeading() {
    return shooterHeading;
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
  public double getHoodPose() {
    return hoodMotor.getAbsoluteEncoder().getPosition();
  }

  /**
   * Configures the motor controllers for the shooter.
   * We use default configuration other than the one motor being a follower, but more can be added
   */
  public void configureSparks() {
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
 * setting the servo hood value
 * @param value returning the servo hood value
 */
  public void setHoodVelocity(double value) {
    hoodMotor.set(value);
  }

  public double calculate(Limelight limelight) {
    double hoodPosition = 0;
    double distance = limelight.getZ();
    hoodPosition = .59 - (.565 * distance) + (.151 * (distance * distance));
    return hoodPosition;
    
  }

  

  @Override
  public void periodic() {
    if(shooterRunning) {
      flywheelEffort = velocityController.calculate(getVelocity(), currentSetpoint) - feedForeward.calculate(getVelocity(), currentSetpoint);
      
      //flywheelEffort *= ShooterConstants.baseEffort;

      double error = velocityController.getError();
      flywheelEffort += .000075 * error; //TODO: Turn this into real feed forward
      //I was an idiot here trying to rush add feed forward and instead just added another factor of proportion
      //Also I tried looking around more and couldnt find anything on velocity controller object but I swore I had seen it before?

      flywheelEffort = MathUtil.clamp(flywheelEffort, -1, 1);
    } else {
      flywheelEffort = 0;
    }
    SmartDashboard.putNumber("Hood Encoder", getHoodPose());
    SmartDashboard.putNumber("Flywheel effort", flywheelEffort);
    SmartDashboard.putNumber("Flywheel Velocity", (getVelocity()));
    SmartDashboard.putNumber("FeedForeward", feedForeward.calculate(getVelocity(), ShooterConstants.baseVelocity));
    SmartDashboard.putBoolean("Close Enough?", closeEnough);
    shooterLeader.set(-flywheelEffort);
    //shooterLeader.setVoltage(4);

  }
}
