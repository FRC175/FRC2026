package frc.robot.subsystems.Drive;

import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;


import frc.robot.utils.Vector;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.Pigeon2;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

public final class AltSwerve implements Subsystem {


    // These variables are final because they only need to be instantiated once (after all, you don't need to create a
    // new left master TalonSRX).
    AltSwerveModule frontRight, frontLeft, backRight, backLeft;
    double lastValidAngle, prevYaw;

    public double x, y, velo;

    Pigeon2 pigeon;

    Translation2d frontRightLocation, frontLeftLocation, backRightLocation, backLeftLocation;
    SwerveDriveKinematics kinematics;
    SwerveDriveOdometry odometry;
    Pose2d pose;
  
    Field2d field;
    SparkFlexConfig defaultConfig;
    ResetMode resetMode;
    PersistMode persistMode;
   
   
    
    /**
     * The single instance of {@link AltSwerve} used to implement the "singleton" design pattern. A description of the
     * singleton design pattern can be found in the JavaDoc for {@link Drive::getInstance()}.
     */
    private static AltSwerve instance;
    
    private AltSwerve() {

        this.defaultConfig = new SparkFlexConfig();
        // leftMaster = new CANSparkMax(DriveConstants.LEFT_MASTER_PORT, CANSparkMaxLowLevel.MotorType.kBrushless);
        frontRight = new AltSwerveModule(DriveConstants.frDriveID, DriveConstants.frTurnID, 45, false, false);
        frontLeft = new AltSwerveModule(DriveConstants.flDriveID, DriveConstants.flTurnID,  135, false, false);
        backRight = new AltSwerveModule(DriveConstants.brDriveID, DriveConstants.brTurnID, -315, false, false);
        backLeft = new AltSwerveModule(DriveConstants.blDriveID, DriveConstants.blTurnID, - 225, false, false);
        configureSparks();

        pigeon = new Pigeon2(10);
        resetGyro(0);
       
        field = new Field2d();

        // SwerveModule test = new SwerveModule

        pose = new Pose2d(0, 0, new Rotation2d());

        final double HALF_WHEEL_BASE_WIDTH = 282575; // meters

       

        frontRightLocation = new Translation2d(HALF_WHEEL_BASE_WIDTH, -HALF_WHEEL_BASE_WIDTH);
        frontLeftLocation = new Translation2d(HALF_WHEEL_BASE_WIDTH, HALF_WHEEL_BASE_WIDTH);
        backRightLocation = new Translation2d(-HALF_WHEEL_BASE_WIDTH, -HALF_WHEEL_BASE_WIDTH);
        backLeftLocation = new Translation2d(-HALF_WHEEL_BASE_WIDTH, HALF_WHEEL_BASE_WIDTH);

        kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);
        odometry = new SwerveDriveOdometry(
            kinematics, new Rotation2d(Math.toRadians(getOdometryAngle())),
            new SwerveModulePosition[] {
              new SwerveModulePosition(0, new Rotation2d()),
              new SwerveModulePosition(0, new Rotation2d()),
              new SwerveModulePosition(0, new Rotation2d()),
              new SwerveModulePosition(0, new Rotation2d())
            }, pose);

        lastValidAngle = 0;
    
    
    defaultConfig.inverted(false);
    //defaultConfig.openLoopRampRate(.05);
    defaultConfig.idleMode(IdleMode.kBrake);

        
  
    SmartDashboard.putData("field", field);
        resetDistance();
    }

    /**
     * <code>getInstance()</code> is a crucial part of the "singleton" design pattern. This pattern is used when there
     * should only be one instance of a class, which makes sense for Robot subsystems (after all, there is only one
     * drivetrain). The singleton pattern is implemented by making the constructor private, creating a static variable
     * of the type of the object called <code>instance</code>, and creating this method, <code>getInstance()</code>, to
     * return the instance when the class needs to be used.
     *
     * Usage:
     * <code>Drive drive = Drive.getInstance();</code>
     *
     * @return The single instance of {@link AltSwerve}
     */
    public static AltSwerve getInstance() {
        if (instance == null) {
            instance = new AltSwerve();
        }

        return instance;
    }

    /**
     * Helper method that configures the Spark Max motor controllers.
     */
    private void configureSparks() {
        frontLeft.configureSparks(defaultConfig, resetMode, persistMode);
        frontRight.configureSparks(defaultConfig, resetMode, persistMode);
        backLeft.configureSparks(defaultConfig, resetMode, persistMode);
        backRight.configureSparks(defaultConfig, resetMode, persistMode);
    }

    public Rotation2d getGyroRot2d() {
        return new Rotation2d(Math.toRadians(getOdometryAngle()));
    }
    public Pigeon2 getGyro() {
        return pigeon;
    }

    /**
     * Sets the drive motors to a certain percent output (demand) using open loop control (no sensors in feedback loop).
     *
     * @param leftDemand The percent output for the left drive motors
     * @param rightDemand The percent output for the right drive motors
     */
    public void setOpenLoop(double drive, double turn) {
        frontRight.driveOpenLoop(drive);
        frontLeft.driveOpenLoop(drive);
        backRight.driveOpenLoop(drive);
        backLeft.driveOpenLoop(drive);

        frontRight.turnOpenLoop(turn);
        frontLeft.turnOpenLoop(turn);
        backRight.turnOpenLoop(turn);
        backLeft.turnOpenLoop(turn);
    }

    @Override
    public void periodic() {

        field.setRobotPose(pose);

        // Get the rotation of the robot from the gyro.
        var gyroAngle = pigeon.getRotation2d();
        

        // System.out.println(gyroAngle.getDegrees());

        // Update the pose
        SmartDashboard.putBooleanArray("modules", new Boolean[]  {
            frontLeft.isReversed(), frontRight.isReversed(), backLeft.isReversed(), backRight.isReversed()
        });

    
        
       //SmartDashboard.putNumber("driveDistance", backLeft.getAbsEncoder());

        // pose = odometry.update(new Rotation2d(Math.toRadians(getOdometryAngle())),
        // new SwerveModulePosition[] {
        //     new SwerveModulePosition(frontLeft.getDriveDistance(), new Rotation2d(Math.toRadians(frontLeft.getOdometryAngle()))), new SwerveModulePosition(frontRight.getDriveDistance(), new Rotation2d(Math.toRadians(frontRight.getOdometryAngle()))),
        //     new SwerveModulePosition(backLeft.getDriveDistance(), new Rotation2d(Math.toRadians(backLeft.getOdometryAngle()))), new SwerveModulePosition(backRight.getDriveDistance(), new Rotation2d(Math.toRadians(backRight.getOdometryAngle())))
        // });

       

        // System.out.println("FL: " + frontLeft.getDriveDistance() + "\t\t" + frontLeft.isReversed());
        // System.out.println("FR: " + frontRight.getDriveDistance() + "\t\t" + frontRight.isReversed());
        // System.out.println("BL: " + backLeft.getDriveDistance() + "\t\t" + backLeft.isReversed());
        // System.out.println("BR: " + backRight.getDriveDistance() + "\t\t" + backRight.isReversed());

        // SmartDashboard.putNumber("X Position: ", getPose().getX());
        // SmartDashboard.putNumber("Y Position: ", getPose().getY());
        // SmartDashboard.putNumber("Front Left Raw", frontLeft.getOdometryAngle());
        //SmartDashboard.putNumber("Toddler Mode Level: ", Constants.TODDLER_MODE);


        // SmartDashboard.putNumber("Distance: ", frontLeft.getDriveDistance());

        postYaw();

    }

    public void setPrevYaw(double value) {
        prevYaw = value;
    } 

    public void resetPose(Pose2d pose) {
        odometry.resetPosition(new Rotation2d(Math.toRadians(getOdometryAngle())),
            new SwerveModulePosition[] {
            new SwerveModulePosition(frontLeft.getDriveDistance(), new Rotation2d(Math.toRadians(frontLeft.getOdometryAngle()))), new SwerveModulePosition(frontRight.getDriveDistance(), new Rotation2d(Math.toRadians(frontRight.getOdometryAngle()))),
            new SwerveModulePosition(backLeft.getDriveDistance(), new Rotation2d(Math.toRadians(backLeft.getOdometryAngle()))), new SwerveModulePosition(backRight.getDriveDistance(), new Rotation2d(Math.toRadians(backRight.getOdometryAngle())))
        }, pose);
    }

    

    // public ChassisSpeeds[] getRobotRelativeSpeeds() {
    //     return new ChassisSpeeds[] {new ChassisSpeeds()}
    // }

    public void swerve(double joyX, double joyY, double twist, double gyroAngle) {
        Vector transversal = new Vector(joyX, joyY * -1);
        // SmartDashboard.putNumber("Joystick Angle", transversal.getAngle());

        if (transversal.getMagnitude() < 0.001) {
            transversal.setAngle(lastValidAngle);
        } else {
            lastValidAngle = transversal.getAngle();
        }

        transversal.rotate(- 90 - gyroAngle);

        frontRight.calculateRawOutputs(transversal, frontRight.findControlledRotationVector(twist));
        frontLeft.calculateRawOutputs(transversal, frontLeft.findControlledRotationVector(twist));
        backRight.calculateRawOutputs(transversal, backRight.findControlledRotationVector(twist));
        backLeft.calculateRawOutputs(transversal, backLeft.findControlledRotationVector(twist));

        frontRight.setOutputs();
        frontLeft.setOutputs();
        backRight.setOutputs();
        backLeft.setOutputs();
    }

    public void swerve(ChassisSpeeds speeds) {
        AltSwerveModule[] modules = new AltSwerveModule[]{frontRight, frontLeft, backRight, backLeft}; 
        speeds = ChassisSpeeds.fromRobotRelativeSpeeds(speeds, getGyroRot2d());
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
        for (int i = 0; i < 4; i++) {
            x = Math.cos(moduleStates[i].angle.getRadians());
            y = Math.sin(moduleStates[i].angle.getRadians());
            velo = moduleStates[i].speedMetersPerSecond;
            Vector transversal = new Vector(x, y);
            modules[i].calculateRawOutputs(transversal, new Vector(0, 0));
        }



    }

    public void lockSwerve(double joyX, double joyY, double lockAngle, double gyroAngle) {
        Vector transversal = new Vector(joyX, joyY * -1);
        // SmartDashboard.putNumber("Joystick Angle", transversal.getAngle());

        if (transversal.getMagnitude() < 0.001) {
            transversal.setAngle(lastValidAngle);
        } else {
            lastValidAngle = transversal.getAngle();
        }

        transversal.rotate(- 90 - gyroAngle);

        frontRight.calculateRawOutputs(transversal, frontRight.findLockedRotationVector(lockAngle, gyroAngle));
        frontLeft.calculateRawOutputs(transversal, frontLeft.findLockedRotationVector(lockAngle, gyroAngle));
        backRight.calculateRawOutputs(transversal, backRight.findLockedRotationVector(lockAngle, gyroAngle));
        backLeft.calculateRawOutputs(transversal, backLeft.findLockedRotationVector(lockAngle, gyroAngle));

        frontRight.setOutputs();
        frontLeft.setOutputs();
        backRight.setOutputs();
        backLeft.setOutputs();
    }

    public void lock() {
        frontRight.lock();
        frontLeft.lock();
        backRight.lock();
        backLeft.lock();

        frontRight.setOutputs();
        frontLeft.setOutputs();
        backRight.setOutputs();
        backLeft.setOutputs();
    }

    public void turning(double fr, double fl, double br, double bl) {
        frontRight.turnOpenLoop(fr);
        frontLeft.turnOpenLoop(fl);
        backRight.turnOpenLoop(br);
        backLeft.turnOpenLoop(bl);
    }

    public double getFRPosition() {
        return frontRight.getDriveEncoder();
    }
    public double getFLPosition() {
        return frontLeft.getDriveDistance();
    }
    public double getBRPosition() {
        return backRight.getDriveEncoder();
    }
    public double getBLPosition() {
        return backLeft.getDriveEncoder();
    }

    public double getDriveDistance() {
        return frontLeft.getDriveDistance();
    }

    public void resetDriveDistance() {
        frontLeft.resetDriveEncoder();
    }

    public double getYaw() {
        return ((pigeon.getYaw().getValueAsDouble() % 360) + 360) % 360;
    }

    public double getRawYaw() {
        return pigeon.getYaw().getValueAsDouble();
    }

    public double getPrevYaw() {
        return prevYaw;
    }

    public double getOdometryAngle() {
        if (getYaw() > 180) {
            return -(360-getYaw());
        } else {
            return getYaw();
        }
    }

    public void resetGyro(double val) {
        pigeon.setYaw(val);
    }

    public void postYaw() {
        SmartDashboard.putNumber("Yee-Yaw", getYaw());
    }

    public void resetDistance() {
        frontLeft.resetDriveEncoder();
    }

    public Pose2d getPose() {
        return pose;
    }

    
    public Supplier<Pose2d> getPoseSup() {
        Supplier<Pose2d> pose = () -> (getPose());
        return pose;

    }

    private ChassisSpeeds getRobotRelativeSpeeds() {

        SwerveModuleState[] moduleStates = new SwerveModuleState[] {
            new SwerveModuleState(frontRight.getVelocity(), new Rotation2d(Math.toRadians(frontRight.getOdometryAngle()))),
            new SwerveModuleState(frontLeft.getVelocity(), new Rotation2d(Math.toRadians(frontLeft.getOdometryAngle()))),
            new SwerveModuleState(backRight.getVelocity(), new Rotation2d(Math.toRadians(backRight.getOdometryAngle()))),
            new SwerveModuleState(backLeft.getVelocity(), new Rotation2d(Math.toRadians(backLeft.getOdometryAngle())))
          };

          return ChassisSpeeds.fromFieldRelativeSpeeds(kinematics.toChassisSpeeds(moduleStates), getGyroRot2d());
    }
    
    

    

   
}