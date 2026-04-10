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
public class AimDutyCycleAuto extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final Shooter shooter;


  private double distance;
  private double heading;
  private double setpoint;
  private double currHoodPose;
 

  // private final PIDController hoodController;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AimDutyCycleAuto(Shooter shooter, Limelight limelight) {
    this.shooter = shooter;
   

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(limelight);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    currHoodPose = shooter.getHoodPose();

    // SmartDashboard.putString("RUNNI", "init");


  setpoint = .79;


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currHoodPose = shooter.getHoodPose();
    heading = shooter.getHeading();
   
    

  setpoint = .405;

  

   // SmartDashboard.putNumber("disttohub", distance);

    //SmartDashboard.putString("RUNNI", "AAAAAAAAAAAAAAAAAAA");
    if (setpoint < ShooterConstants.maxHoodExtension) {
      if (currHoodPose < setpoint - .01) {
        shooter.setHoodVelocity(.2);
      } else if(currHoodPose > setpoint + .01){
        shooter.setHoodVelocity(-.2);
      } else shooter.setHoodVelocity(0);
    }}

  

  // 0 is just a placeholder//
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setHoodVelocity(0);
    //SmartDashboard.putBoolean("INTERUPPTED", interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (currHoodPose <= setpoint + .01 && currHoodPose >= setpoint - .01);
  }
}
