// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.SwerveJoystick;
import frc.robot.subsystems.Climb;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Drive.Swerve;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  //The robot's subsystems
  private final Climb m_climb = new Climb();
  private final Shooter m_shooter = new Shooter();
  private final Swerve m_Drive = new Swerve();

 
  //Drive team controllers
  private final CommandXboxController m_driverController =
      new CommandXboxController(DriveConstants.driverControllerPort);

  
  public RobotContainer() {

    
    // Configure the trigger bindings
   
    m_Drive.setDefaultCommand(new SwerveJoystick(m_Drive, () -> m_driverController.getLeftX(), () ->  m_driverController.getLeftY(), () -> m_driverController.getRightX(), () -> !m_driverController.a().getAsBoolean()));
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    /** Button Binding Templates **
    
    //Press Button: Do action once when the button is pressed
    new Trigger(() -> m_driverController.getAButton()).onTrue(
      new InstantCommand(() -> m_subsystem.method())
    );
    
    //Hold Button: Do action while held, optional other action when not held
    new Trigger(() -> m_driverController.getAButton()).whileTrue(
      new InstantCommand(() -> m_subsystem.method())
    ).whileFalse(
      new InstantCommand(() -> m_subsystem.method())
    ));

    */

    // //Hold B: Climb motor at 6.25% speed
    // new Trigger(() -> m_driverController.getLeftX() == 1).whileTrue(
    //   new InstantCommand(() -> flt.set(.5))
    // ).whileFalse(
    //   new InstantCommand(() ->  flt.set(0))
    // );
    //Hold A: To set shooter velocity at 5.50% clockwise
    // new Trigger(() -> m_driverController.getAButton()).whileTrue(
    //   new InstantCommand(() -> m_shooter.setShooterVelocity( .0550 )) 
    // ).whileFalse(
    // new InstantCommand(() -> m_shooter.setShooterVelocity( 0))
    // );
    // //Hold X: To set shooter velocity at 5.50% counterclockwise
    // new Trigger(() -> m_driverController.getXButton()).whileTrue(
    //   new InstantCommand(() -> m_shooter.setShooterVelocity( -.0550 )) 
    // ).whileFalse(
    // new InstantCommand(() -> m_shooter.setShooterVelocity( 0))
    // );

    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  //   // An example command will be run in autonomous
  //   return Autos.exampleAuto(m_climb);
  // }
}
