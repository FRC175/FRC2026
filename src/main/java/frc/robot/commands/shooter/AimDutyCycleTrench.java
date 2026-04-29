// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;

/** An example command that uses an example subsystem. */
public class AimDutyCycleTrench extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final Shooter shooter;

  private double distance;
  private double hoodPosition;
  private double heading;
  private double setpoint;

  private Timer timer;

  // private final PIDController hoodController;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AimDutyCycleTrench(Shooter shooter) {
    this.shooter = shooter;
    timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    setpoint = .374;
    SmartDashboard.putString("hood state", "running");


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //Find the distance 

    double targetHoodSetpoint = .374; //Calculate hood position
    
    double voltage = shooter.hoodController.calculate(shooter.getHoodPose(), targetHoodSetpoint); //Calculate voltage output from PID

    if (setpoint < ShooterConstants.maxHoodExtension) {
      shooter.setHoodVoltage(voltage);
      SmartDashboard.putBoolean("toofar", false);
    } else {
      SmartDashboard.putBoolean("toofar", true);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setHoodVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false ;
  }
}