// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;

/** An example command that uses an example subsystem. */
public class Aim extends Command  {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final Shooter shooter;
  private final Limelight limelight;
  
  private double distance;
  private double hoodPosition;
  private double heading;
  private double setpoint;

  private Timer timer;
  //private final PIDController hoodController;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Aim(Shooter shooter, Limelight limelight, double setpoint) {
    this.shooter = shooter;
    this.limelight = limelight;
    timer = new Timer();
    this.setpoint = setpoint;
    this.setpoint = setpoint;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    heading = shooter.getHeading();
    distance = limelight.getZtoHub(heading);
    SmartDashboard.putNumber("Limelight distance", distance);
    double targetHoodSetpoint = setpoint;
    double effort = shooter.hoodController.calculate(shooter.getHoodPose(), targetHoodSetpoint);
    if (setpoint < ShooterConstants.maxHoodExtension) {
    shooter.setHoodVelocity(effort);
     SmartDashboard.putBoolean("toofar", false);
    } else {
      SmartDashboard.putBoolean("toofar", true);
    }

    
    
  }
  //0 is just a placeholder//
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setHoodVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
    
}
}