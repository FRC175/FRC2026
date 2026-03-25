// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Shooter;

import javax.print.attribute.SetOfIntegerSyntax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;

/** An example command that uses an example subsystem. */
public class AimDutyCycle extends Command  {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final Shooter shooter;
  
  private double distance;
  private double hoodPosition;
  private double heading;
  private double setpoint;

  private boolean up;

  private Timer timer;
  //private final PIDController hoodController;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AimDutyCycle(Shooter shooter, double setpoint, boolean up) {
    this.shooter = shooter;
    timer = new Timer();
    this.setpoint = setpoint;
    this.up = up;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
SmartDashboard.putString("RUNNI", "init");

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Hoodsetpoint", setpoint);
    
    SmartDashboard.putString("RUNNI", "AAAAAAAAAAAAAAAAAAA");
    if (setpoint < ShooterConstants.maxHoodExtension) {
      if (up) {
      shooter.setHoodVelocity(.1);
    } else {
      shooter.setHoodVelocity(-.1);
    }
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
    SmartDashboard.putBoolean("INTERUPPTED", interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
     if (up) {
       SmartDashboard.putString("RUNNI", "done organically");
      return (shooter.getHoodPose() >= setpoint);
    } else {
      SmartDashboard.putString("RUNNI", "done organically");
      return (shooter.getHoodPose() <= setpoint);
    }
  }
}
