// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Shooter;

import javax.print.attribute.SetOfIntegerSyntax;


import javax.print.attribute.SetOfIntegerSyntax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;

/** An example command that uses an example subsystem. */
public class AimDutyCycle extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final Shooter shooter;
  private final Limelight limelight;

  private double distance;
  private double heading;
  private double setpoint;
  private double currHoodPose;
  private boolean override;

  // private final PIDController hoodController;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AimDutyCycle(Shooter shooter, Limelight limelight) {
    this.shooter = shooter;
    this.limelight = limelight;
    this.override = override;

    // Use addRequirements() here to declare subsystem dependencies.
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // currHoodPose = shooter.getHoodPose();

    // // SmartDashboard.putString("RUNNI", "init");

    // heading = shooter.getHeading();
    
    //   distance = limelight.getZtoHub(heading);
    
    
// if (setpoint < ShooterConstants.maxHoodExtension) {
//       if (currHoodPose < setpoint - .001) {
//         shooter.setHoodVelocity(.2);
//       } else if(currHoodPose > setpoint + .001){
//         shooter.setHoodVelocity(-.2);
//       } else shooter.setHoodVelocity(0);
//     }

   }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  double targetHoodSetpoint = .115;
   double voltage = shooter.hoodController.calculate(shooter.getHoodPose(), targetHoodSetpoint); //Calculate voltage output from PID

    if (setpoint < ShooterConstants.maxHoodExtension) {
      shooter.setHoodVoltage(voltage);
      SmartDashboard.putBoolean("toofar", false);
    } else {
      SmartDashboard.putBoolean("toofar", true);
    }
    

  

    //SmartDashboard.putNumber("disttohub", distance);

    SmartDashboard.putString("RUNNI", "running");
    

  }

  // 0 is just a placeholder//
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
     SmartDashboard.putString("RUNNI", "done");
    shooter.setHoodVelocity(0);
    //SmartDashboard.putBoolean("INTERUPPTED", interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (false);
  }
}
