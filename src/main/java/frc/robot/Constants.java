// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
  
  //Constants that pertain to the Drive Team/Robot Control
  public static class OperatorConstants {
    public static final int driverControllerPort = 0;
    public static final int operatorControllerPort = 1;
    public static final int climbControllerPort = 2;

  }

  //Constants that pertain to the Swerve Drivebase
  public static class DriveConstants {

    //Motor ID's
    public static final int frTurnID = 2;
    public static final int frDriveID = 3;
    public static final int flTurnID = 4;
    public static final int flDriveID = 5;
    public static final int blTurnID = 6;
    public static final int blDriveID = 7;
    public static final int brTurnID = 8;
    public static final int brDriveID = 9;

    //Velocity/Acceleration Constants
    public static final double maxSpeed = 5;
    public static final double maxTeleopSpeed = maxSpeed *.75;
    public static final double maxDriveAcceleration = 3;
    public static final double maxAngularVelocity = Math.PI;
    public static final double maxTeleopAngularSpeed = maxAngularVelocity / 4;
    public static final double maxAngularAcceleration = 3;

    //Robot Dimensions and Gear Ratio Conversions
    public static final double wheelDiameter = Units.inchesToMeters(4);
    public static final double driveGearRatio = 1 / 6.75;
    public static final double turnGearRatio = 1 / 21.43;
    public static final double driveEncoderResolution = driveGearRatio * Math.PI * wheelDiameter;
    public static final double turnEncoderResolution = turnGearRatio * 2 * Math.PI;
    public static final double driveSpeedResolution = driveEncoderResolution / 60;
    public static final double turnSpeedResolution = turnEncoderResolution / 60;

    //Swerve Module Kinematic Constants
    public static final Translation2d frontLeftLocation = new Translation2d(0.282575, 0.282575);
    public static final Translation2d frontRightLocation = new Translation2d(0.282575, -0.282575);
    public static final Translation2d backLeftLocation = new Translation2d(-0.282575, 0.282575);
    public static final Translation2d backRightLocation = new Translation2d(-0.282575, -0.282575);
    
    public static final double pTurnConstants = .2;
    public static final double iTurnConstants = 0;
    public static final double dTurnConstants = 0;

    public static final double pDriveConstants = 1;
    public static final double iDriveConstants = 0;
    public static final double dDriveConstants = 0;

    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

    public static final double kPXController = 1.5;
    public static final double kPYController = 1.5;
    public static final double kPThetaController = 3;

    public static final TrapezoidProfile.Constraints kThetaControllerContraints = 
      new TrapezoidProfile.Constraints(maxTeleopAngularSpeed, maxTeleopAngularSpeed/10);
  }

  //Constants that pertain to the Intake subsystem
  public static class IntakeConstants{
    //Motor ID's
    public static final int deployID = 16;
    public static final int rollerID = 17;

    public static enum intakeState {
      Stowed,
      Travel,
      Deployed,
      Intermediary
    }

    //Encoder position for deploying or retracting intake (in reference to previous position, in terms of rotations)
    public static final double intakeRetractPosition = 25;
    public static final double intakeMiddlePosition = 61.5;
    public static final double intakeDeployPosition = 145;

    public static final double intakeSpeed = -.6;

  }

  //Constants that pertain to the Shooter subsystem
  public static class ShooterConstants{
    //Motor ID's
    public static final int shooterLeaderID = 11;
    public static final int shooterFollowID = 12;

    //Hood Servos
    public static final int leftHoodServo = 0;
    public static final int rightHoodServo = 1;

    public static final double FrontHubAngle = 0;

    public static final double baseVelocity = 3750;
    public static final double baseEffort = .45;

    public static final double passingVelocity = 4000;
    public static final double FrontHubSpeed = 2600;
    //locked in

    public static final double[] blueShooterIDs= new double[] {5, 8, 9, 10, 11, 2};
    public static final double[] redShooterIDs= new double[] {21, 24, 25, 26, 27, 18};
  }

  //Constants that pertain to the Hopper subsystem
  public static class HopperConstants{
    //Motor ID's
    public static final int feederWheelID = 13; //Smaller green gummy wheel above fuel as enters shooter
    public static final int feederTrackID = 14; //Gummy and mechanum wheels in feeder floor
    public static final int spinDexerID = 15; //Central rotor at bottom of hopper
    
  }

  //Constants that pertain to the Climb subsystem
  public static class ClimbConstants{
    public static final int climbID = 20;
    public static final double climbPos = -108;
    public static final double climbMin = -4.76;

  }

}
