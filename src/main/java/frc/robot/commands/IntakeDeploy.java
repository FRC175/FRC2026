// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;

/** A command that deploys the Intake */
public class IntakeDeploy extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final Intake intake; 
 
  public IntakeDeploy(Intake intake) {
    this.intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setDeployPosition(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setDeployVelocity(.01);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setDeployVelocity(0);
  }

  // 300 is a dummy value.
  @Override
  public boolean isFinished() {
    if(intake.getDeployPosition() >= IntakeConstants.intakeDeployPosition) {
    return true;
   } else return false;
  }
}
