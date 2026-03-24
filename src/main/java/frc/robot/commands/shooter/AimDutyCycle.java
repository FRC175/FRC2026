// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Shooter;
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

  private Timer timer;
  //private final PIDController hoodController;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AimDutyCycle(Shooter shooter,double setpoint) {
    this.shooter = shooter;
    timer = new Timer();
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
    SmartDashboard.putString("RUNNI", "AAAAAAAAAAAAAAAAAAA");
    shooter.setHoodVelocity(.05);
    
  }
  //0 is just a placeholder//
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setHoodVelocity(0);
    SmartDashboard.putString("RUNNI", "nope");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (shooter.getHoodPose() >= setpoint);
}
}