// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AimThenShoot;
import frc.robot.commands.Autos;
import frc.robot.commands.SwerveJoystick;
import frc.robot.subsystems.Climb;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import java.util.List;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Drive.Swerve;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
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
  private final Swerve drive;
  private final Climb climb;
  private final Shooter shooter;
  private final Hopper hopper;
  private final Intake intake;
  private final Limelight limelight;

  //The drive team controllers are defined
  private final XboxController driverController = new XboxController(OperatorConstants.driverControllerPort);
  private final XboxController operatorController = new XboxController(OperatorConstants.operatorControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    this.drive = Swerve.getInstance();
    this.climb = Climb.getInstance();
    this.shooter = Shooter.getInstance();
    this.hopper = Hopper.getInstance();
    this.intake = Intake.getInstance();
    this.limelight = new Limelight();
    
    /**
     * Setting default commands for each subsystem that 
     * -Must ambiently run in some way, such as a mechanism resisting being pushed either direction
     * -Teleop controls that require more than a Trigger can 
     */
    drive.setDefaultCommand(new SwerveJoystick(
        drive, 
        () -> driverController.getLeftX(), 
        () ->  driverController.getLeftY(), 
        () -> driverController.getRightX(), 
        () -> !driverController.getAButton())
    );

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
    
    new Trigger(() -> driverController.getAButton()).whileTrue (
      new AimThenShoot(shooter, limelight, hopper)
    ).onFalse(
      new SequentialCommandGroup(
      new InstantCommand(() -> shooter.stop()),
      new InstantCommand(() -> hopper.stop()),
      new InstantCommand(() -> shooter.setServoHood(0)))
    );

    new Trigger(() -> driverController.getYButton()).onTrue(
      new InstantCommand(() -> shooter.setServoHood(.1))
    );

    new Trigger(() -> driverController.getBButton()).whileTrue(
      new InstantCommand(() -> shooter.setVelocity(-.3))
    ).onFalse(
      new InstantCommand(() -> shooter.stop())
    );
    
    new Trigger(() -> driverController.getLeftBumperButton()).onTrue(
      new InstantCommand(() -> intake.setDeployPosition(0))
    );

    new Trigger(() -> driverController.getRightBumperButton()).whileTrue(
      new InstantCommand(() -> shooter.setServoHood(.5))
    ).onFalse(
      new InstantCommand(() -> shooter.setServoHood(0))
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
      DriveConstants.maxSpeed, 
      DriveConstants.maxDriveAcceleration)
      .setKinematics(DriveConstants.kinematics);

      Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(
          new Translation2d(1, 0),
          new Translation2d(1, -1)
        ),
        new Pose2d(2, -1, Rotation2d.fromDegrees(100)), trajectoryConfig
        );

        PIDController xController = new PIDController(DriveConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(DriveConstants.kPXController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
          DriveConstants.kPThetaController, 0, 0, DriveConstants.kThetaControllerContraints
          );
          thetaController.enableContinuousInput(-Math.PI, Math.PI);
          SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(trajectory, drive::getPose, DriveConstants.kinematics, xController, yController, thetaController, drive::setModuleStates, drive);

    
    return new SequentialCommandGroup(
      new InstantCommand(() -> drive.resetPose(trajectory.getInitialPose())), 
      swerveControllerCommand, 
      new InstantCommand(() -> drive.stopModules())
    );
  }
}
