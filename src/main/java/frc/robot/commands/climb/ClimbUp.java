// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import frc.robot.subsystems.ExampleSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climb;
import edu.wpi.first.wpilibj.Timer;


/** An example command that uses an example subsystem. */
public class ClimbUp extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final Climb m_Climb;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ClimbUp(Climb climb) {
    m_Climb = climb;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    m_Climb.climbSpeed(true, .3);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Climb.climbSpeed(true, 0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_Climb.getPosition() <= Constants.ClimbConstants.climbPos);
  }
}
