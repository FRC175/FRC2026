// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriverConstants;
import frc.robot.subsystems.Drive.SwerveModule;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class Swerve extends SubsystemBase {

    private final Translation2d frontRightLocation = new Translation2d(0.381, -0.381);
    private final Translation2d frontLeftLocation = new Translation2d(0.381, 0.381);
    private final Translation2d backRightLocation = new Translation2d(-0.381, -0.381);
    private final Translation2d backLeftLocation = new Translation2d(-0.381, 0.381);
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontRightLocation, frontLeftLocation, backRightLocation, backLeftLocation);

    private final SwerveModule frontRight = new SwerveModule(1, 2, 3, false, false, 0, false);
    private final SwerveModule frontLeft = new SwerveModule(4, 5, 6, false, false, 0, false);
    private final SwerveModule backRight = new SwerveModule(7, 8, 9, false, false, 0, false);
    private final SwerveModule backLeft = new SwerveModule(10, 11, 12, false, false, 0, false);
    private final Pigeon2 gyro = new Pigeon2(10, "CANivore_BUS");

    /** Creates a new Swerve System. */
    public Swerve() {

        resetGyro();

    }

    public void resetGyro() {
        gyro.reset();
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getRotation2d().getRadians(), 360);
    }

    public Rotation2d getRotation2d() {
        return gyro.getRotation2d();
    }

    public void stopModules() {
        frontRight.stop();
        frontLeft.stop();
        backRight.stop();
        backLeft.stop();
    }

    public void setModuleStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriverConstants.maxDriveSpeed);
        frontRight.setDesiredState(states[0]);
        frontLeft.setDesiredState(states[1]);
        backRight.setDesiredState(states[2]);
        backLeft.setDesiredState(states[3]);
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
