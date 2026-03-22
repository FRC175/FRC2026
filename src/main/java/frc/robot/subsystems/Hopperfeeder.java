package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants.HopperConstants;

public class Hopperfeeder extends SubsystemBase {
    private final Hopperfeeder instance;
    private final SparkFlex rotary, feederWheel;
    private final RelativeEncoder rotaryEncoder, feederTrackEncoder, feederWheelEncoder;
    private final SparkMax feederTrack;

  /**
   * Creates a new Hopperfeeder Subsystem
   */
  public Hopperfeeder() {
    rotary = new SparkFlex(HopperConstants.spinDexerID, MotorType.kBrushless);
    rotaryEncoder = rotary.getEncoder();

    feederTrack = new SparkMax(HopperConstants.feederTrackID, MotorType.kBrushless);
    feederTrackEncoder = feederTrack.getEncoder();

    feederWheel = new SparkFlex(HopperConstants.feederWheelID, MotorType.kBrushless);
    feederWheelEncoder = feederWheel.getEncoder();
  }

  /**
   * Returns the initialized Hopperfeeder subsystem, or creates a Hopperfeeder if there is not one already
   * @return The current Hopperfeeder instance
   */
  public static Hopperfeeder getInstance() {
    if (instance == null) {
      instance = new Hopperfeeder();
    }
    return instance;
  }

 //Sets speed for rotary
  public void setHopperVelocity(double speed){
    rotary.set(speed);
    feederTrack.set(speed);
    feederWheel.set(speed);
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