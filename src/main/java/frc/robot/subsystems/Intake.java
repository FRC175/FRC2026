package frc.robot.subsystems;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.intakeState;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

public class Intake extends SubsystemBase {
  private static Intake instance;
  private final SparkMax intakeDeploy;
  private final AbsoluteEncoder deployEncoder;
  private final SparkMax intakeRoller;
  private final RelativeEncoder rollerEncoder;
  public final PIDController pid;

  private intakeState state = intakeState.Stowed;

  /**
   * Creates a new intake subsystem
   */
  public Intake() {
    intakeDeploy = new SparkMax(IntakeConstants.deployID, MotorType.kBrushless);
    deployEncoder = intakeDeploy.getAbsoluteEncoder();
    intakeRoller = new SparkMax(IntakeConstants.rollerID, MotorType.kBrushless);
    rollerEncoder = intakeRoller.getEncoder();

    pid = new PIDController(.015, .005, 0);
    pid.setTolerance(5);

    correctState();
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
   * Sets the intake state to the declared state
   * @param newState New state the intake has been set to
   */
  public void setState(intakeState newState) {
    this.state = newState;
  }

  /**
   * Returns the current state of the intake deployment
   * @return The state of the intake (Stowed, Travel Position, Deployed)
   */
  public intakeState getState() {
    return state;
  }

  /**
   * Checks the current position of the intake encoder, and sets the state based on its location
   */
  private void correctState() {
    double currPosition = getAbsolutePosition();

    if(currPosition >= 140) state = intakeState.Deployed;
    else if(currPosition <= 25) state = intakeState.Stowed;
    else state = intakeState.Travel;
    
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
  // public void setDeployPosition(double position) {
  //   deployEncoder.reset;
  // }

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

  public double getAbsolutePosition() {
    return deployEncoder.getPosition();
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
    SmartDashboard.putNumber("Position", getAbsolutePosition());
    switch (state) {
      case Deployed:
        SmartDashboard.putString("State:", "Deployed");
        break;
      case Travel:
        SmartDashboard.putString("State:", "Travel");
        break;
      case Stowed:
        SmartDashboard.putString("State:", "Stowed");

    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
