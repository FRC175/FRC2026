package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  private static Intake instance;
  private final SparkMax intakeDeploy;
  private final RelativeEncoder deployEncoder;
  private final SparkMax intakeRoller;
  private final RelativeEncoder rollerEncoder;

  /**
   * Creates a new intake subsystem
   */
  public Intake() {
    intakeDeploy = new SparkMax(IntakeConstants.deployID, MotorType.kBrushless);
    deployEncoder = intakeDeploy.getEncoder();
    intakeRoller = new SparkMax(IntakeConstants.rollerID, MotorType.kBrushless);
    rollerEncoder = intakeRoller.getEncoder();
  }

  /**
   * Returns the initialized intake subsystem, or creates an intake if there is not one already
   * @return The current intake instance
   */
  public static Intake getInstance() {
    if(instance == null) {
      instance = new Intake();
    }
    return instance;
  }

  /**
   * Sets the motor velocity of the deploy motor
   * @param speed Motor velocity
   */
  public void setDeployVelocity(double speed) {
    intakeDeploy.set(speed);

  }

  /**
   * Sets the deploy encoder reading
   * @param position Desired encoder reading
   */
  public void setDeployPosition(double position) {
    intakeDeploy.set(position);
  }

  /**
   * Sets the speed of the intake rollers
   * @param speed
   */
  public void setRollerSpeed(double speed) {
    intakeRoller.set(speed);
  }

  /**
   * Retrieves the current velocity of the deploy motor
   * @return Deploy motor velocity
   */
  public double getDeployVelocity() {
    double deployReading = deployEncoder.getVelocity();
    return deployReading;
  }

  /**
   * Retrieves the current deploy encoder reading
   * @return Current deploy encoder reading (units of rotations)
   */
  public double getDeployPosition() {
    double deployReading = deployEncoder.getPosition() / 25;
    return deployReading;
  }

  /**
   * Retrieves the current velocity of the rollers
   * @return Current roller velocity
   */
  public double getRollerVelocity() {
    double rollerVelocity = rollerEncoder.getVelocity();
    return rollerVelocity;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
