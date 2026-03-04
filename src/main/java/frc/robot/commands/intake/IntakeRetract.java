// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import frc.robot.subsystems.Intake;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;

/** A command that deploys the Intake */
public class IntakeRetract extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final Intake intake;
  private final PIDController retractController;
  private double retractEffort;
 
  public IntakeRetract(Intake intake) {
    this.intake = intake;

    retractController = new PIDController(.5, 0, 0);
    retractController.setSetpoint(IntakeConstants.intakeRetractPosition);
    retractController.setTolerance(1);
    retractEffort = 0;
    
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    retractEffort = MathUtil.clamp(retractController.calculate(intake.getAbsolutePosition(), IntakeConstants.intakeRetractPosition), -1, 1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    retractEffort = MathUtil.clamp(retractController.calculate(intake.getAbsolutePosition(), IntakeConstants.intakeRetractPosition), -1, 1);
    //retractEffort *= IntakeConstants.intakeSpeed;
    intake.setDeployVelocity(retractEffort);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setDeployVelocity(0);
  }

  // 300 is a dummy value.
  @Override
  public boolean isFinished() {
    return retractController.atSetpoint();
  }
}
