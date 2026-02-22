// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This csass should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int driverControllerPort = 0;
    public static final int operatorControllerPort = 1;
  }
  public static class IntakeConstants{
    //Motor ID's
    public static final int deployID = 16;
    public static final int rollerID = 17;

    //Encoder position for deploying or retracting intake (in reference to previous position)
    public static final double intakeDeployPosition = 300;
    public static final double intakeRetractPosition = -300;
  }

  public static class ShooterConstants{
    //Motor ID's
    public static final int shooterLeaderID = 11;
    public static final int shooterFollowID = 12;

    //Hood Servos
    public static final int leftHoodServo = 9;
    public static final int rightHoodServo = 8;

    //Nominal flywheel speed at 75% of full power
    public static final double flyWheelEffort = .75;
  }

  public static class HopperConstants{
    //Motor ID's
    public static final int feederWheelID = 13; //Smaller green gummy wheel above fuel as enters shooter
    public static final int feederTrackID = 14; //Gummy and mechanum wheels in feeder floor
    public static final int spinDexerID = 15; //Central rotor at bottom of hopper
    
  }

  public static class ClimbConstants{
    public static final int climbID = 16;
  }
}
