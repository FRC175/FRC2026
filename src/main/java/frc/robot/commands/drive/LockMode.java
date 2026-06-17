// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drive.Swerve;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LockMode extends Command {

  private final Swerve drive;
  SwerveModuleState[] moduleStates = {null, null, null, null};
  /** Creates a new LockMode. */
  public LockMode(Swerve drive) {
    this.drive = drive;
   addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SwerveModuleState fL = new SwerveModuleState(0, new Rotation2d(45));
    SwerveModuleState fR = new SwerveModuleState(0, new Rotation2d(-45)); 
    SwerveModuleState bL = new SwerveModuleState(0, new Rotation2d(45));
    SwerveModuleState bR = new SwerveModuleState(0, new Rotation2d(-45));
    
    moduleStates[0] = fL;
    moduleStates[1] = fR;
    moduleStates[2] = bL;
    moduleStates[3] = bR;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.setModuleStates(moduleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
