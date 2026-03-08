// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ClimbConstants;

import frc.robot.commands.SwerveJoystick;
import frc.robot.commands.climb.ClimbDown;
import frc.robot.commands.climb.ClimbUp;
import frc.robot.commands.intake.IntakeForewards;
import frc.robot.commands.intake.IntakeMiddle;
import frc.robot.commands.intake.IntakeBackwards;
import frc.robot.commands.shooter.Aim;
import frc.robot.commands.shooter.AimThenShoot;
import frc.robot.commands.shooter.Shoot;

import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Drive.Swerve;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

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

  // The drive team controllers are defined
  private final XboxController driverController = new XboxController(OperatorConstants.driverControllerPort);
  private final XboxController operatorController = new XboxController(OperatorConstants.operatorControllerPort);
  private final XboxController climbController = new XboxController(OperatorConstants.climbControllerPort);

  private final SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    this.drive = Swerve.getInstance();
    this.climb = Climb.getInstance();
    this.shooter = Shooter.getInstance();
    this.hopper = Hopper.getInstance();
    this.intake = Intake.getInstance();
    this.limelight = Limelight.getInstance();

    autoChooser = new SendableChooser<>();


    /**
     * Setting default commands for each subsystem that
     * -Must ambiently run in some way, such as a mechanism resisting being pushed
     * either direction
     * -Teleop controls that require more than a Trigger can
     */
    drive.setDefaultCommand(new SwerveJoystick(
        drive,
        () -> -driverController.getLeftY(),
        () -> -driverController.getLeftX(),
        () -> -driverController.getRightX(),
        () -> !driverController.getAButton()));

    configureBindings();
    configureAutoChooser();

    
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

    
    new Trigger(() -> operatorController.getRightTriggerAxis() == 1).whileTrue(
        new AimThenShoot(shooter, limelight, hopper)).onFalse(
            new SequentialCommandGroup(
                new InstantCommand(() -> shooter.stop()),
                new InstantCommand(() -> hopper.stop()),
                new InstantCommand(() -> shooter.setServoHood(0))));

     new Trigger(() -> operatorController.getLeftTriggerAxis() == 1).whileTrue(
        new SequentialCommandGroup (
            new InstantCommand(() -> shooter.setServoHood(ShooterConstants.FrontHubAngle)),
            new Shoot(shooter, ShooterConstants.FrontHubSpeed),
            new InstantCommand(() -> hopper.run())
        )).onFalse(
            new SequentialCommandGroup(
                new InstantCommand(() -> shooter.stop()),
                new InstantCommand(() -> hopper.stop()),
                new InstantCommand(() -> shooter.setServoHood(0)))
                );


     new Trigger(() -> operatorController.getLeftBumperButton()).whileTrue(
        new SequentialCommandGroup(
            new InstantCommand(() -> shooter.setServoHood(.9)),
            new Shoot(shooter, ShooterConstants.passingVelocity))
        
        ).onFalse (
            new SequentialCommandGroup(
                new InstantCommand(() -> shooter.stop()),
                new InstantCommand(() -> shooter.setServoHood(0))));
    
                
    new Trigger(() -> operatorController.getRightBumperButton()).whileTrue(
            new InstantCommand(() -> hopper.run())
        
        ).onFalse (
                new InstantCommand(() -> hopper.stop())
        );
     

     


    new Trigger(() -> operatorController.getXButton()).onTrue(
        new IntakeForewards(intake));

     new Trigger(() -> driverController.getLeftBumperButton()).onTrue(
        new InstantCommand(() -> drive.setGyro(0)));


    new Trigger(() -> operatorController.getBButton()).onTrue(
        new IntakeBackwards(intake));
        
    new Trigger(() -> operatorController.getYButton()).whileTrue(
        new InstantCommand(() -> intake.setRollerSpeed(IntakeConstants.intakeSpeed))).onFalse(
            new InstantCommand(() -> intake.setRollerSpeed(0)));

    new Trigger(() -> operatorController.getAButton()).onTrue(
        new IntakeMiddle(intake));

    new Trigger(() -> operatorController.getPOV() == 0).onTrue(
        //new ClimbUp(climb, .1));
        new InstantCommand(() -> climb.setSpeed(-.4))).onFalse(
          new InstantCommand(() -> climb.setSpeed(0))
        );

    new Trigger(() -> operatorController.getPOV() == 180).onTrue(
        //new ClimbDown(climb, .1));
        new InstantCommand(() -> climb.setSpeed(.4))).onFalse(
          new InstantCommand(() -> climb.setSpeed(0))
        );


    new Trigger(() -> climbController.getPOV() == 0).whileTrue(
        new InstantCommand(() -> climb.setSpeed(-.1))).whileFalse(
            new InstantCommand(() -> climb.setSpeed(0)));

    // new Trigger(() -> climbController.getPOV() == 0).whileTrue(
    //     new InstantCommand(() -> climb.zeroEncoder()));

        new Trigger(() -> climbController.getPOV() == 180).whileTrue(
        new InstantCommand(() -> climb.setSpeed(.1))).whileFalse(
            new InstantCommand(() -> climb.setSpeed(0)));


    new Trigger(() -> climbController.getAButton()).onTrue(
      new IntakeForewards(intake)
    );

    new Trigger(() -> climbController.getBButton()).onTrue(
      new IntakeMiddle(intake)
    );

    new Trigger(() -> climbController.getRightStickButton()).onTrue(
      new InstantCommand(() -> shooter.setServoHood(.5))).onFalse(
        new InstantCommand(() -> shooter.setServoHood(0))
      );
        new Trigger(() -> operatorController.getRightTriggerAxis() == 1).whileTrue(
        new AimThenShoot(shooter, limelight, hopper)).onFalse(
            new SequentialCommandGroup(
                new InstantCommand(() -> shooter.stop()),
                new InstantCommand(() -> hopper.stop()),
                new InstantCommand(() -> shooter.setServoHood(0))));

     new Trigger(() -> climbController.getLeftTriggerAxis() == 1).whileTrue(
        new SequentialCommandGroup (
            new InstantCommand(() -> shooter.setServoHood(ShooterConstants.FrontHubAngle)),
            new Shoot(shooter, ShooterConstants.FrontHubSpeed),
            new InstantCommand(() -> hopper.run())
        )).onFalse(
            new SequentialCommandGroup(
                new InstantCommand(() -> shooter.stop()),
                new InstantCommand(() -> hopper.stop()),
                new InstantCommand(() -> shooter.setServoHood(0)))
                );


     new Trigger(() ->  climbController.getLeftBumperButton()).whileTrue(
        new SequentialCommandGroup(
            new InstantCommand(() -> shooter.setServoHood(.9)),
            new Shoot(shooter, ShooterConstants.passingVelocity))
        
        ).onFalse (
            new SequentialCommandGroup(
                new InstantCommand(() -> shooter.stop()),
                new InstantCommand(() -> shooter.setServoHood(0))));
    
                
    new Trigger(() -> climbController.getRightBumperButton()).whileTrue(
            new InstantCommand(() -> hopper.run())
        
        ).onFalse (
                new InstantCommand(() -> hopper.stop())
        );
     

     


    new Trigger(() -> climbController.getXButton()).onTrue(
        new IntakeForewards(intake));


    new Trigger(() -> climbController.getBButton()).onTrue(
        new IntakeBackwards(intake));
        
    new Trigger(() -> climbController.getYButton()).whileTrue(
        new InstantCommand(() -> intake.setRollerSpeed(IntakeConstants.intakeSpeed))).onFalse(
            new InstantCommand(() -> intake.setRollerSpeed(0)));

    new Trigger(() -> climbController.getAButton()).onTrue(
        new IntakeMiddle(intake));

    
    // new Trigger(() -> climbController.getYButton()).onTrue(
    //   new IntakeBackwards(intake)
    // );
//ay int rev
// a and b <- -> 1 position
  }

  private void configureAutoChooser() {
    autoChooser.setDefaultOption("Preload", new ParallelCommandGroup(
        new IntakeMiddle(intake),
        new SequentialCommandGroup(
            new InstantCommand(() -> drive.setGyro(90))), 
            new AimThenShoot(shooter, limelight, hopper)
        ));
    autoChooser.addOption("Preload on the starting line", new ParallelCommandGroup(
        new IntakeMiddle(intake),
        new SequentialCommandGroup (
            new InstantCommand(() -> drive.setGyro(90)),
            new InstantCommand(() -> shooter.setServoHood(0)),
            new Shoot(shooter, ShooterConstants.FrontHubSpeed),
            new InstantCommand(() -> hopper.run()),
            new WaitCommand(10),
            new InstantCommand(() -> shooter.stop()),
            new InstantCommand(() -> hopper.stop())

        )));
    autoChooser.addOption("Nothing", new ParallelCommandGroup(new WaitCommand(0)));
   
    SmartDashboard.putData(autoChooser);
  
  };
    
   
    

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    return autoChooser.getSelected();
    
  
    // trajectory settings
    // TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
    //     DriveConstants.maxSpeed,
    //     DriveConstants.maxDriveAcceleration)
    //     .setKinematics(DriveConstants.kinematics);

    // // create trajectory
    // Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
    //     new Pose2d(0, 0, new Rotation2d(0)),
    //     List.of(
    //         new Translation2d(1, 0),
    //         new Translation2d(1, -1)),
    //     new Pose2d(2, -1, Rotation2d.fromDegrees(180)), trajectoryConfig);
    // // create PID controllers
    // PIDController xController = new PIDController(DriveConstants.kPXController, 0, 0);
    // PIDController yController = new PIDController(DriveConstants.kPXController, 0, 0);
    // ProfiledPIDController thetaController = new ProfiledPIDController(
    //     DriveConstants.kPThetaController, 0, 0, DriveConstants.kThetaControllerContraints);
    // thetaController.enableContinuousInput(-Math.PI, Math.PI);
    // // create command
    // SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
    //     trajectory, drive::getPose, DriveConstants.kinematics, xController, yController, thetaController,
    //     drive::setModuleStates, drive);

    // return new SequentialCommandGroup(
    //     new InstantCommand(() -> drive.resetPose(trajectory.getInitialPose())),
    //     swerveControllerCommand,
    //     new InstantCommand(() -> drive.stopModules()));
    
  }
}
