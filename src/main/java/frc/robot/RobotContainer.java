// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;

import frc.robot.commands.climb.ClimbDown;
import frc.robot.commands.climb.ClimbUp;
import frc.robot.commands.drive.AngleToLime;
import frc.robot.commands.drive.SwerveJoystick;
import frc.robot.commands.intake.Agitate;
import frc.robot.commands.intake.IntakeDeploy;
import frc.robot.commands.intake.IntakeTravel;
import frc.robot.commands.intake.MaintainPosition;
import frc.robot.commands.intake.IntakeRetract;
import frc.robot.commands.shooter.Aim;
import frc.robot.commands.shooter.AimDutyCycle;
import frc.robot.commands.shooter.AimDutyCycleAuto;
import frc.robot.commands.shooter.AimDutyCycleTrench;
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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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

        NamedCommands.registerCommand("Shoot from hub", new Shoot(shooter, ShooterConstants.FrontHubSpeed));
        NamedCommands.registerCommand("Start Shooter", new InstantCommand(() -> shooter.run()));
        NamedCommands.registerCommand("Prep Shooter", new SequentialCommandGroup(new InstantCommand(() -> shooter.velocityController.reset()), new InstantCommand(() -> shooter.currentSetpoint = ShooterConstants.baseVelocity), new InstantCommand(() -> shooter.velocityController.setSetpoint(ShooterConstants.baseVelocity))));
        
        NamedCommands.registerCommand("Stop Shooter", new InstantCommand(() -> shooter.stop()));
        NamedCommands.registerCommand("Aim", new AimDutyCycle(shooter, limelight));
        NamedCommands.registerCommand("Reset Gyro 180", new InstantCommand(() -> drive.setGyro(180)));
        NamedCommands.registerCommand("Stop Shooter", new InstantCommand(() -> shooter.stop()));
        NamedCommands.registerCommand("Spindexer", new InstantCommand(() -> hopper.run()));
        NamedCommands.registerCommand("Stop Intake", new InstantCommand(() -> intake.setRollerSpeed(IntakeConstants.intakeSpeed)));
        NamedCommands.registerCommand("Stop Spindexer", new InstantCommand(() -> hopper.stop()));
        NamedCommands.registerCommand("Deploy Intake", new IntakeDeploy(intake));
        NamedCommands.registerCommand("Mid Intake", new IntakeTravel(intake));
        NamedCommands.registerCommand("Run Intake",
                new InstantCommand(() -> intake.setRollerSpeed(IntakeConstants.intakeSpeed)));
        NamedCommands.registerCommand("Stop Intake", new InstantCommand(() -> intake.setRollerSpeed(0)));
        NamedCommands.registerCommand("Climb Up", new ClimbUp(climb));
        NamedCommands.registerCommand("Climb Down", new ClimbDown(climb));
        NamedCommands.registerCommand("Angle to Lime", new AngleToLime(drive, limelight));
        NamedCommands.registerCommand("Aim Trench Auto", new AimDutyCycleAuto(shooter));

        /**
         * Setting default commands for each subsystem that
         * -Must ambiently run in some way, such as a mechanism resisting being pushed
         * either direction
         * -Teleop controls that require more than a simple trigger
         */

        drive.setDefaultCommand(new SwerveJoystick(
                drive,
                limelight,
                shooter,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> -driverController.getRightX(),
                () -> !driverController.getAButton(),
                () -> driverController.getRightBumperButton(),
                () -> (driverController.getRightTriggerAxis() >= .75)));
        intake.setDefaultCommand(new MaintainPosition(intake));

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
        // DRIVER CONTROLLER BINDINGS
        /**
         * Left Bumper - Reset Bot Orientation (Intake Facing Away From Driver Station)
         */
        new Trigger(() -> driverController.getLeftBumperButton()).onTrue(
                new InstantCommand(() -> drive.setGyro(0)));

        // OPERATOR CONTROLLER BINDINGS
        // ** Right Trigger - Run Intake */
        new Trigger(() -> operatorController.getRightTriggerAxis() == 1).whileTrue(
                new InstantCommand(() -> intake.setRollerSpeed(IntakeConstants.intakeSpeed))).onFalse(
                        new InstantCommand(() -> intake.setRollerSpeed(0)));

        // ** Left Trigger - Shoot from Hub */
        new Trigger(() -> operatorController.getLeftTriggerAxis() == 1).whileTrue(
                new SequentialCommandGroup(
                        //new InstantCommand(() -> shooter.setHoodNeo(.01)),
                        new Shoot(shooter, ShooterConstants.FrontHubSpeed),
                        new InstantCommand(() -> hopper.run())))
                .onFalse(
                        new ParallelCommandGroup(
                                new InstantCommand(() -> shooter.stop()),
                                new InstantCommand(() -> hopper.stop())));

        // ** Left Bumper - Run Shooter and Set Hood */
        new Trigger(() -> operatorController.getLeftBumperButton()).whileTrue(
                new ParallelCommandGroup(
                        
                        new Shoot(shooter, ShooterConstants.baseVelocity)
                        )).onFalse(
                            new SequentialCommandGroup(
                                new InstantCommand(() -> hopper.stop()),
                                new WaitCommand(1),
                                new InstantCommand(() -> shooter.stop())));

        // ** Right Bumper - Run Hopper */
        new Trigger(() -> operatorController.getRightBumperButton()).whileTrue(
                new SequentialCommandGroup(
                new AimDutyCycle(shooter, limelight),
                new WaitCommand(.2),
                new InstantCommand(() -> hopper.run()),
                new WaitCommand(3),
                new InstantCommand(() -> intake.setRollerSpeed(IntakeConstants.intakeSpeed)),
                new WaitCommand(.3),
                new InstantCommand(() -> intake.setRollerSpeed(0)),
                new IntakeTravel(intake),
                new WaitCommand(.3),
                new IntakeDeploy(intake),
                new WaitCommand(.3),
                new IntakeTravel(intake),
                new WaitCommand(.3),
                new IntakeDeploy(intake)
                )).onFalse(
                        new InstantCommand(() -> hopper.stop()));

        /** X Button - Retract Intake */
        new Trigger(() -> operatorController.getXButton()).onTrue(
                new IntakeRetract(intake));

        // ** B Button - Deploy Intake */
        new Trigger(() -> operatorController.getBButton()).onTrue(
                new IntakeDeploy(intake));

        // ** A Button - Intake Middle (Travel) Position */
        new Trigger(() -> operatorController.getAButton()).onTrue(
                new IntakeTravel(intake));

        // ** D-Pad Up - Climb Up */
        new Trigger(() -> operatorController.getPOV() == 0).onTrue(
                // new ClimbUp(climb, .1));
                new InstantCommand(() -> climb.climbSpeed(true, .4))).onFalse(
                        new InstantCommand(() -> climb.setSpeed(0)));

        // ** D-Pad Down - Climb Down */
        new Trigger(() -> operatorController.getPOV() == 180).onTrue(
                // new ClimbDown(climb, .1));
                new InstantCommand(() -> climb.climbSpeed(false, .4))).onFalse(
                        new InstantCommand(() -> climb.setSpeed(0)));

        // ** D-Pad Left - Agitate Intake (Subject to Change) */
        new Trigger(() -> operatorController.getPOV() == 270).whileTrue(
                 new SequentialCommandGroup(
                        new AimDutyCycleTrench(shooter),
                        new InstantCommand(() -> hopper.run())
                        )).onFalse(
                            new SequentialCommandGroup(
                            new InstantCommand(() -> hopper.stop())));

        
                // new ClimbDown(climb, .1));
        

        // CLIMB CONTROLLER (Do we still need?)
        new Trigger(() -> climbController.getPOV() == 0).whileTrue(
                new InstantCommand(() -> climb.setSpeed(-.1))).whileFalse(
                        new InstantCommand(() -> climb.setSpeed(0)));

        // new Trigger(() -> climbController.getPOV() == 0).whileTrue(
        // new InstantCommand(() -> climb.zeroEncoder()));

        new Trigger(() -> climbController.getPOV() == 180).whileTrue(
                new InstantCommand(() -> climb.setSpeed(.1))).whileFalse(
                        new InstantCommand(() -> climb.setSpeed(0)));

        /*
         * new Trigger(() -> climbController.getAButton()).onTrue(
         * new AimDutyCycle(shooter, .7, true));
         * 
         * new Trigger(() -> climbController.getYButton()).onTrue(
         * new AimDutyCycle(shooter, .15, false));
         */

        new Trigger(() -> climbController.getLeftTriggerAxis() == 1).whileTrue(
                new SequentialCommandGroup(
                        // new InstantCommand(() ->
                        // shooter.setServoHood(ShooterConstants.FrontHubAngle)),
                        new Shoot(shooter, ShooterConstants.FrontHubSpeed),
                        new InstantCommand(() -> hopper.run())))
                .onFalse(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> shooter.stop()),
                                new InstantCommand(() -> hopper.stop())
                        // new InstantCommand(() -> shooter.setServoHood(0)))
                        ));

        new Trigger(() -> climbController.getLeftBumperButton()).onTrue(
                new ClimbUp(climb)

        );

        new Trigger(() -> climbController.getRightBumperButton()).onTrue(
                new ClimbDown(climb)

        );

        new Trigger(() -> climbController.getXButton()).onTrue(
                new IntakeDeploy(intake));

        new Trigger(() -> climbController.getBButton()).onTrue(
                new IntakeRetract(intake));

        // new Trigger(() -> climbController.getYButton()).whileTrue(
        // new InstantCommand(() ->
        // intake.setRollerSpeed(IntakeConstants.intakeSpeed))).onFalse(
        // new InstantCommand(() -> intake.setRollerSpeed(0)));

        new Trigger(() -> climbController.getAButton()).whileTrue(
        new InstantCommand(() -> climb.setSpeed(.2))).onFalse(
            new InstantCommand(() -> climb.setSpeed(0))
        );

        new Trigger(() -> driverController.getAButton()).onTrue(
                new InstantCommand(() -> drive.resetDistModules()));

        // new Trigger(() -> climbController.getYButton()).onTrue(
        // new IntakeBackwards(intake)
        // );
        // ay int rev
        // a and b <- -> 1 position
    }

    private void configureAutoChooser() {
        // autoChooser.setDefaultOption("Preload", new ParallelCommandGroup(
        // new IntakeTravel(intake),
        // new SequentialCommandGroup(
        // new InstantCommand(() -> drive.setGyro(90))),
        // new DriveFor(drive, .5),
        // new AimThenShoot(shooter, limelight, hopper)
        // ));
        // autoChooser.addOption("drive .5m forewards", new DriveFor(drive, 0.5));
        autoChooser.addOption("Preload on the starting line", new ParallelCommandGroup(
                new IntakeTravel(intake),
                new SequentialCommandGroup(
                        new InstantCommand(() -> drive.setGyro(90)),
                        // new InstantCommand(() -> shooter.setServoHood(0)),
                        new Shoot(shooter, ShooterConstants.FrontHubSpeed),
                        new InstantCommand(() -> hopper.run()),
                        new WaitCommand(10),
                        new InstantCommand(() -> shooter.stop()),
                        new InstantCommand(() -> hopper.stop())

                )));
        autoChooser.addOption("Nothing", new ParallelCommandGroup(new WaitCommand(0)));

        autoChooser.addOption("ligne then shoot", new PathPlannerAuto("ligne then SHOOT B)"));
        autoChooser.addOption("Fav (Right Climb)", new PathPlannerAuto("Grimper (faux)"));
        autoChooser.addOption("Sprite Haluchi (Right Trench)", new SequentialCommandGroup(new PathPlannerAuto("shoot Rtrench, mid, shoot"), new InstantCommand(() -> drive.setGyro(180))));
        autoChooser.addOption("Baha Blast (Left Trench)", new PathPlannerAuto("shoot Ltrench, mid, shoot"));
        autoChooser.addOption("Straight to (Right Trench)", new SequentialCommandGroup(new PathPlannerAuto("Rtrench, mid, shoot"), new InstantCommand(() -> drive.setGyro(180))));
        
        // autoChooser.addOption("autoaim", new );

        SmartDashboard.putData(autoChooser);

    };

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {

        return autoChooser.getSelected();

        // TODO: Now that we have the time, revisit '0 To Autonomous'
        // trajectory settings
        // TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
        // DriveConstants.maxSpeed,
        // DriveConstants.maxDriveAcceleration)
        // .setKinematics(DriveConstants.kinematics);

        // // create trajectory
        // Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        // new Pose2d(0, 0, new Rotation2d(0)),
        // List.of(
        // new Translation2d(1, 0),
        // new Translation2d(1, -1)),
        // new Pose2d(2, -1, Rotation2d.fromDegrees(180)), trajectoryConfig);
        // // create PID controllers
        // PIDController xController = new PIDController(DriveConstants.kPXController,
        // 0, 0);
        // PIDController yController = new PIDController(DriveConstants.kPXController,
        // 0, 0);
        // ProfiledPIDController thetaController = new ProfiledPIDController(
        // DriveConstants.kPThetaController, 0, 0,
        // DriveConstants.kThetaControllerContraints);
        // thetaController.enableContinuousInput(-Math.PI, Math.PI);
        // // create command
        // SwerveControllerCommand swerveControllerCommand = new
        // SwerveControllerCommand(
        // trajectory, drive::getPose, DriveConstants.kinematics, xController,
        // yController, thetaController,
        // drive::setModuleStates, drive);

        // return new SequentialCommandGroup(
        // new InstantCommand(() -> drive.resetPose(trajectory.getInitialPose())),
        // swerveControllerCommand,
        // new InstantCommand(() -> drive.stopModules()));

    }
}
