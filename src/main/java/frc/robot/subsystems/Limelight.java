// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight extends SubsystemBase {

  private static Limelight instance;
  private final NetworkTable table;

  /** Creates a new Limelight */
  public Limelight() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
  }

  /**
   * Returns the initialized limelight, or creates a limelight if there is not one already
   * @return The current limelight instance
   */
  public static Limelight getInstance() {
    if(instance == null) {
      instance = new Limelight();
    }
    return instance;
  }

  /**
   * Method to retrieve the horizontal offset from the crosshair to the target.
   * 
   * @return the horizontal offset in degrees
   */
  private double getTx() {
    return table.getEntry("tx").getDouble(0.0);
  }

  /**
   * Method to retrieve the vertical offset from the crosshair to the target.
   * 
   * @return the vertical offset in degrees
   */
  private double getTy() {
    return table.getEntry("ty").getDouble(0.0);
  }

  /**
   * Method to retrieve the target area.
   * 
   * @return the target area as a percentage of the image
   */
  private double getTa() {
    return table.getEntry("ta").getDouble(0.0);
  }

  /**
   * Method to determine if the limelight has found a target.
   * 
   * @return true if a target is found, false otherwise
   */
  private boolean foundTarget() {

    return (table.getEntry("tv").getDouble(0.0) == 1);

  }

  /**
   * Method to determine if the limelight has found a target, and returns its 2D
   * pose if so.
   * 
   * @return an array of doubles representing [tx, ty, ta] if a target is found,
   *         null otherwise
   */
  public double[] findAprilTag2D() {

    if (foundTarget() == false) {

      return null;

    } else {

      double tX = getTx();
      double tY = getTy();
      double tA = getTa();

      return new double[] { tX, tY, tA };
    }
  }

  /**
   * Method to determine if the limelight has found a target, and returns its 3D
   * pose in robot space if so.
   * 
   * @return an array of doubles representing the 3D pose [x, y, z, roll, pitch,
   *         yaw] in robot space if a target is found, null otherwise
   */
  public double[] findAprilTag3D() {

    if (foundTarget() == false)
      return null;

    else
      return table.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);

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
