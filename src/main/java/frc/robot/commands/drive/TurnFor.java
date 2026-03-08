// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drive.Swerve;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class TurnFor extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final Swerve swerve;
  private double goalAngle, currAngle;
  private Pose2d startPos, currPose;

  /**
   * Turns robot by a set angle 
   *
   * @param swerve The subsystem used by this command.
   * @param goalAngle the desired angle to turn by (degrees)
   */
  public TurnFor(Swerve swerve, double goalAngle) {
    this.swerve = swerve;
    this.goalAngle = goalAngle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startPos = swerve.getPose();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currPose = swerve.getPose();
    currAngle = currPose.getRotation().getDegrees() - startPos.getRotation().getDegrees();

    ChassisSpeeds speed = new ChassisSpeeds(0, 0, .25 * Math.PI);
    SwerveModuleState[] swerveStates = DriveConstants.kinematics.toSwerveModuleStates(speed);
    swerve.setModuleStates(swerveStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (currAngle >= goalAngle);
  }
}
