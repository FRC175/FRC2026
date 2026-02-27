// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Drive.SwerveModule;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.Constants.DriveConstants;

public class Swerve extends SubsystemBase {

    private static Swerve instance;

    private final SwerveModule frontLeft = new SwerveModule(DriveConstants.flDriveID, DriveConstants.flTurnID, false, false,  false);
    private final SwerveModule frontRight = new SwerveModule(DriveConstants.frDriveID, DriveConstants.frTurnID, false, false,  false);
    private final SwerveModule backLeft = new SwerveModule(DriveConstants.blDriveID, DriveConstants.blTurnID, false, false, false);
    private final SwerveModule backRight = new SwerveModule(DriveConstants.brDriveID, DriveConstants.brTurnID, false, false,  false);

    private final Pigeon2 gyro = new Pigeon2(10);
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kinematics, new Rotation2d(0), new SwerveModulePosition[] {frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()} );

    /** Creates a new Swerve System. */
    public Swerve() {

        resetGyro();
        

    }

    /**
   * Returns the initialized shooter subsystem, or creates a shooter if there is not one already
   * @return The current shooter instance
   */
  public static Swerve getInstance() {
    if(instance == null) {
      instance = new Swerve();
    }
    return instance;
  }

    /**
     * Offsets the Gyro by 90 degrees to the left (adjust for mounting)
     */
    public void resetGyro() {
        gyro.setYaw(270);
    }

    /**
     * Gets the heading of the robot in radians
     * 
     * @return The current robot heading (radians)
     */
    public double getHeading() {
        return Math.IEEEremainder(gyro.getRotation2d().getRadians(), 2 * Math.PI);
    }

    /**
     * Gets the current heading of the robot as a rotation2d
     * 
     * @return The robot's current heading (rotation2d)
     */
    public Rotation2d getRotation2d() {
        return gyro.getRotation2d();
    }

    
    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }
     public void resetPose(Pose2d pose) {
        odometer.resetPose(pose);;
    }

    /**
     * Stops all 4 swerve modules with individual method
     */
    public void stopModules() {
        frontRight.stop();
        frontLeft.stop();
        backRight.stop();
        backLeft.stop();
    }

    /**
     * Sets the states of each swerve module, after being normalized according to
     * the max speed
     * 
     * @param states Array of the desired swerve states ([frontLeft, frontRight,
     *               backLeft, backRight])
     */
    public void setModuleStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.maxSpeed);
        frontLeft.setDesiredState(states[0]);
        frontRight.setDesiredState(states[1]);
        backLeft.setDesiredState(states[2]);
        backRight.setDesiredState(states[3]);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("angle pos", backRight.getAbsoluteEncoderRad());
        odometer.update(getRotation2d(), new SwerveModulePosition[] {frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()});
       
        
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
