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
public class IntakeForewards extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final Intake intake; 
  private double deployEffort;
  
 
  public IntakeForewards(Intake intake) {
    this.intake = intake;
    deployEffort = 0;
  
    addRequirements(intake);
  }
  //deploy this 9:14 3/6/26  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    if (intake.getState() == intakeState.Travel) {
      intake.pid.setSetpoint(IntakeConstants.intakeDeployPosition); 
     } else if (intake.getState() == intakeState.Stowed) {
       intake.pid.setSetpoint(IntakeConstants.intakeMiddlePosition);
     }
      
    }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    deployEffort = MathUtil.clamp(intake.pid.calculate(intake.getAbsolutePosition()), -1, 1);

    deployEffort *= .175;

    SmartDashboard.putNumber("DeployEffort", deployEffort);

    intake.setDeployVelocity(deployEffort);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setDeployVelocity(0);
    if (intake.getState() == intakeState.Travel) {
      intake.setState(intakeState.Deployed); 
    } else if (intake.getState() == intakeState.Stowed) {
      intake.setState(intakeState.Travel);
    }
  }

  // 300 is a dummy value.
  @Override
  public boolean isFinished() {
   return intake.pid.atSetpoint();
}
}
