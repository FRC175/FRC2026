// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import frc.robot.subsystems.Intake;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.intakeState;


/** A command that deploys the Intake */
public class IntakeDeploy extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final Intake intake; 
  private double effort;
  
 
  public IntakeDeploy(Intake intake) {
    this.intake = intake;
    effort = 0;
  
    addRequirements(intake);
  }
  //deploy this 9:14 3/6/26  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    intake.pid.reset();
    intake.pid.setSetpoint(IntakeConstants.intakeDeployPosition); 

    }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    effort = MathUtil.clamp(intake.pid.calculate(intake.getAbsolutePosition()), -1, 1);
    effort *= .175;
    //SmartDashboard.putNumber("DeployEffort", deployEffort);
    intake.setDeployVelocity(effort);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setState(intakeState.Deployed); 
  }

  // 300 is a dummy value.
  @Override
  public boolean isFinished() {
   return intake.pid.atSetpoint();
}
}
