package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import frc.robot.Constants.HopperConstants;

public class Hopper extends SubsystemBase {
    private static Hopper instance;
    private final SparkFlex rotary, feederWheel;
    private final RelativeEncoder rotaryEncoder, feederTrackEncoder, feederWheelEncoder;
    private final SparkMax feederTrack;

  /**
   * Creates a new Hopper Subsystem
   */
  public Hopper() {
    rotary = new SparkFlex(HopperConstants.spinDexerID, MotorType.kBrushless);
    rotaryEncoder = rotary.getEncoder();

    feederTrack = new SparkMax(HopperConstants.feederTrackID, MotorType.kBrushless);
    feederTrackEncoder = feederTrack.getEncoder();

    feederWheel = new SparkFlex(HopperConstants.feederWheelID, MotorType.kBrushless);
    feederWheelEncoder = feederWheel.getEncoder();
  }

  /**
   * Returns the initialized Hopper subsystem, or creates a Hopper if there is not one already
   * @return The current Hopper instance
   */
  public static Hopper getInstance() {
    if (instance == null) {
      instance = new Hopper();
    }
    return instance;
  }

  public void run(){
    rotary.set(.125);
    feederTrack.set(.3);
    feederWheel.set(.3);
  }

  public void stop(){
    rotary.set(0);
    feederTrack.set(0);
    feederWheel.set(0);
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