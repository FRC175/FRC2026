// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Drive.Swerve;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class AngleToLime extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final Swerve swerve;
  private final Limelight limelight;
  private double goalAngle, currAngle;
  private double currentAngle;
  private final PIDController turnController;
  private final Supplier<Double> xSpeedFunction, ySpeedFunction;
 
  private final SlewRateLimiter xLimiter, yLimiter;

  /**
   * Turns robot by a set angle 
   *
   * @param swerve The subsystem used by this command.
   * @param goalAngle the desired angle to turn by (degrees)
   */
  public AngleToLime(Swerve swerve, Limelight limelight, Supplier<Double> xSpeedFunction, Supplier<Double> ySpeedFunction) {
    this.swerve = swerve;
    this.limelight = limelight;

    turnController = new PIDController(DriveConstants.pTurnConstants, 0, 0);
    turnController.setTolerance(.1);
    //Swerve subsystem
        

        //Input functions
        this.xSpeedFunction = xSpeedFunction;
        this.ySpeedFunction = ySpeedFunction;
       
       

        //Rate Limiters
        this.xLimiter = new SlewRateLimiter(DriveConstants.maxDriveAcceleration);
        this.yLimiter = new SlewRateLimiter(DriveConstants.maxDriveAcceleration);
        

        addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    

    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Gets the joystick inputs
        double xSpeed = xSpeedFunction.get();
        SmartDashboard.putNumber("LIMEx-Stick", xSpeed);
        double ySpeed = ySpeedFunction.get();
         SmartDashboard.putNumber("LIMEy-Stick", ySpeed);
        

        //Apply the Deadband
        xSpeed = Math.abs(xSpeed) > .1 ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > .1 ? ySpeed : 0.0;
        
        //Rate Limiter on joysticks and scale to 75% of max speed for Teleop
        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.maxTeleopSpeed;
        SmartDashboard.putNumber("LIMEConverted X Speed", xSpeed);
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.maxTeleopSpeed;
        SmartDashboard.putNumber("LIMEConverted Y Speed", ySpeed);
        
        //SmartDashboard.putNumber("Converted Turn Speed", );

    currentAngle = limelight.getTx(26);
    double effort = turnController.calculate(currentAngle, 0);
    SmartDashboard.putNumber("turning effort", effort);
    //new SwerveJoystick(swerve, null, null, null, null)
    ChassisSpeeds speed = new ChassisSpeeds(xSpeed, ySpeed , effort);
    SwerveModuleState[] swerveStates = DriveConstants.kinematics.toSwerveModuleStates(speed);
    swerve.setModuleStates(swerveStates);
    SmartDashboard.putString("auto aiming?", "AIMING....");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("auto aiming?", "nope");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (turnController.atSetpoint());
  }
}
