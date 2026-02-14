// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class DriveConstants {
    public static final int driverControllerPort = 0;
    public static final double maxSpeed = 3;
    public static final double maxTeleopSpeed = maxSpeed / 4;
    public static final double maxDriveAcceleration = 3;
    public static final double maxAngularVelocity = Math.PI;
    public static final double maxTeleopAngularSpeed = maxAngularVelocity / 4;
    public static final double maxAngularAcceleration = 3;

    public static final double wheelDiameter = Units.inchesToMeters(4);
    public static final double driveGearRatio = 1 / 6.75;
    public static final double turnGearRatio = 1 / 21.43;
    public static final double driveEncoderResolution = driveGearRatio * Math.PI * wheelDiameter;
    public static final double turnEncoderResolution = turnGearRatio * 2 * Math.PI;
    public static final double driveSpeedResolution = driveEncoderResolution / 60;
    public static final double turnSpeedResolution = turnEncoderResolution / 60;

    public static final Translation2d frontLeftLocation = new Translation2d(0.282575, 0.282575);
    public static final Translation2d frontRightLocation = new Translation2d(0.282575, -0.282575);
    public static final Translation2d backLeftLocation = new Translation2d(-0.282575, 0.282575);
    public static final Translation2d backRightLocation = new Translation2d(-0.282575, -0.282575);
    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontRightLocation, frontLeftLocation, backRightLocation, backLeftLocation);
  }

  public static class IntakeConstants {
    public static final double setEncoderPosition = 300;
  }

}
