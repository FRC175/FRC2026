// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Drive.Swerve;
import edu.wpi.first.wpilibj2.command.Command;

/** Retrieves the robot's heading from the swerve base, and updates the shooter's heading accordingly. */
public class UpdateShooterHeading extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final Shooter shooter;
  private final Swerve swerve;

  /**
   * Creates a new UpdateShooterHeadng Command.
   *
   * @param shooter The shooter subsystem.
   * @param swerve The swerve subsystem.
   */
  public UpdateShooterHeading(Shooter shooter, Swerve swerve) {
    this.shooter = shooter;
    this.swerve = swerve;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double botHeading = swerve.getHeading();
    shooter.updateHeading(botHeading);
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
