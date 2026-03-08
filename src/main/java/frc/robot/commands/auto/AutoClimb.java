// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;
import frc.robot.subsystems.Drive.Swerve;
import frc.robot.commands.climb.ClimbDown;
import frc.robot.commands.climb.ClimbUp;
import frc.robot.commands.drive.DriveFor;
import frc.robot.subsystems.Climb;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/** An example command that uses an example subsystem. */
public class AutoClimb extends SequentialCommandGroup  {
  @SuppressWarnings("PMD.UnusedPrivateField")
 
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoClimb(Climb climb, Swerve swerve) {
    addCommands(
        
        new ParallelCommandGroup(
            new ClimbUp(climb),
            new DriveFor(swerve, 1)
        ),
        new ClimbDown(climb)
               
    );
  }

}
