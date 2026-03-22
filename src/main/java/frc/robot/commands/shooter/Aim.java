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
public class Aim extends Command  {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final Shooter shooter;
  private final Limelight limelight;
  
  private double distance;
  private double hoodPosition;
  private double heading;

  private Timer timer;
  private final PIDController hoodController;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Aim(Shooter shooter, Limelight limelight) {
    this.shooter = shooter;
    this.limelight = limelight;
    timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
    hoodController = new PIDController(.01, 0, 0);
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
    double targetHoodSetpoint = 0;
    double effort = hoodController.calculate(shooter.getHoodPose(), targetHoodSetpoint);
    shooter.setHoodVelocity(effort);

    
    
  }
  //0 is just a placeholder//
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return hoodController.atSetpoint();
}
}