// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.shooter.Aim;
import frc.robot.commands.shooter.AimThenShoot;
import frc.robot.commands.shooter.Shoot;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Drive.Swerve;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Limelight;

/** An example command that uses an example subsystem. */
public class ShootPreload extends SequentialCommandGroup  {
  @SuppressWarnings("PMD.UnusedPrivateField")
 
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ShootPreload(Shooter shooter, Limelight limelight, Hopper hopper, Swerve swerve) {
    addCommands(
    new AimThenShoot(shooter,limelight,hopper)
               
    );
  }

}
