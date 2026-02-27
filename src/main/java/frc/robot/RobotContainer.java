// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.Climb;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Hopperfeeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //private final Climb m_climb = new Climb();
  //private final Shooter m_shooter = new Shooter();
  //private final Hopperfeeder m_hopper = new Hopperfeeder();
  //private final Intakedeploy m_deploy = new Intakedeploy();
  //private final Intake m_intake = new Intake();
  private final Limelight m_limelight;


  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final XboxController m_driverController =
      new XboxController(OperatorConstants.driverControllerPort);

  private final Climb climb;
  private final Shooter shooter;
  private static Hopperfeeder hopper;
  private static Intake intake;

  private final XboxController driverController = new XboxController(OperatorConstants.driverControllerPort);
  private final XboxController operatorController = new XboxController(OperatorConstants.operatorControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    this.climb = Climb.getInstance();
    this.shooter = Shooter.getInstance();
    this.hopper = Hopperfeeder.getInstance();
    this.intake = Intake.getInstance();
    // Configure the trigger bindings
    m_limelight = new Limelight();
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    /**
     * Button Binding Templates **
     * 
     * //Press Button: Do action once when the button is pressed
     * new Trigger(() -> driverController.getAButton()).onTrue(
     * new InstantCommand(() -> m_subsystem.method())
     * );
     * 
     * //Hold Button: Do action while held, optional other action when not held
     * new Trigger(() -> driverController.getAButton()).whileTrue(
     * new InstantCommand(() -> m_subsystem.method())
     * ).whileFalse(
     * new InstantCommand(() -> m_subsystem.method())
     * ));
     * 
     * //Todo (Low Priority): Add temlates for triggers and joysticks
     * 
     */

    // Hold B: Climb motor at 6.25% speed

    // Hold A: To set shooter velocity at 5.50% clockwise
    new Trigger(() -> driverController.getAButton()).whileTrue(
        new InstantCommand(() -> shooter.setShooterVelocity(.0550))).whileFalse(
            new InstantCommand(() -> shooter.setShooterVelocity(0)));

    // Hold Y: To set Hopperfeeder velocity at 5.50% clockwise
    new Trigger(() -> driverController.getYButton()).whileTrue(
        new InstantCommand(() -> intake.setDeployVelocity(.1500))).whileFalse(
            new InstantCommand(() -> intake.setDeployVelocity(0)));
    // Press Up on Dpad: set servoHood postition
    new Trigger(() -> driverController.getPOV() == 0).whileTrue(
        new InstantCommand(() -> shooter.setServoHood(.750))).whileFalse(
            new InstantCommand(() -> shooter.setServoHood(0)));
    new Trigger(() -> driverController.getPOV() == 90).whileTrue(
        new InstantCommand(() -> shooter.setServoHood(.250))).whileFalse(
            new InstantCommand(() -> shooter.setServoHood(0)));
    new Trigger(() -> driverController.getPOV() == 180).whileTrue(
        new InstantCommand(() -> shooter.setServoHood(.550))).whileFalse(
            new InstantCommand(() -> shooter.setServoHood(0)));
    new Trigger(() -> driverController.getPOV() == 270).whileTrue(
        new InstantCommand(() -> shooter.setServoHood(999))).whileFalse(
            new InstantCommand(() -> shooter.setServoHood(0)));

  }

  // public Command getAutonomousCommand() {
  // An example command will be run in autonomous
  // return Autos.exampleAuto(m_climb);
  // }
}
