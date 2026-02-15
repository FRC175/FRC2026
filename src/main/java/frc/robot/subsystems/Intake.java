package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

public class Intake extends SubsystemBase {
  private final SparkMax intakeDeployLeader;
  private final RelativeEncoder leaderDeployEncoder;
  // private final SparkMax intakeDeployFollower;
  // private final RelativeEncoder followerDeployEncoder;

  public Intake() {
    // Change deviceId to 1 for testing 6 is a place holder before testing
    intakeDeployLeader = new SparkMax(2, MotorType.kBrushless);
    leaderDeployEncoder = intakeDeployLeader.getEncoder();
    // Change deviceId to 2 for testing 7 is place holder before testing
    // intakeDeployFollower = new SparkMax(7, MotorType.kBrushless);
    // followerDeployEncoder = intakeDeployFollower.getEncoder();
  }

  public void setDeployVelocity(double speed) {
    intakeDeployLeader.set(speed);
    // intakeDeployFollower.set(speed);

  }

  public void setEncoderPosition(double position) {
    intakeDeployLeader.set(position);
  }

  public double getEncoderVelocity() {
    double leaderDeployReading = leaderDeployEncoder.getVelocity();
    // double followerDeployReading = followerDeployEncoder.getVelocity();
    // followerDeployReading *= -1;
    return leaderDeployReading; // (leaderDeployReading + followerDeployReading)/2;
  }

  public double getEncoderPosition() {
    double leaderDeployReading = leaderDeployEncoder.getPosition();
    // double followerDeployReading = followerDeployEncoder.getVelocity();
    // followerDeployReading *= -1;
    return leaderDeployReading; // (leaderDeployReading + followerDeployReading)/2;
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
