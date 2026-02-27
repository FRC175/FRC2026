// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
//import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANcoder;
import frc.robot.Constants.DriveConstants;

public class SwerveModule extends SubsystemBase {

    private final SparkFlex driveMotor;
    private final SparkFlex turnMotor;
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turnEncoder;

    private final boolean driveMotorReversed;
    private final boolean turnMotorReversed;

    private final AbsoluteEncoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    private final PIDController turnPID;


    /** Creates a new SwerveModule. */
    public SwerveModule(int driveMoterID, int turnMotorID,
            boolean driveMotorReversed,
            boolean turnMotorReversed, boolean absoluteEncoderReversed) {

        // Initialize motors
        driveMotor = new SparkFlex(driveMoterID, MotorType.kBrushless);
        turnMotor = new SparkFlex(turnMotorID, MotorType.kBrushless);

        
        
        // Set motor inversion if needed
        this.driveMotorReversed = driveMotorReversed;
        this.turnMotorReversed = turnMotorReversed;

        configureFlexes();

        // Initialize encoders
        driveEncoder = driveMotor.getEncoder();
        turnEncoder = turnMotor.getEncoder();
        // Initialize absolute encoder
        absoluteEncoderOffsetRad = 0;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = turnMotor.getAbsoluteEncoder();

        // Initialize PID controller for turning motor (should only need P)
        turnPID = new PIDController(0.2, 0.0, 0.0);
        turnPID.enableContinuousInput(-1*Math.PI, Math.PI);

        resetEncoders();

    }

    /**
     * Retrieves the position of the drive motor
     * @return Position of the drive motor (meters)
     */
    public double getDrivePosition() {
        return driveEncoder.getPosition() * DriveConstants.driveEncoderResolution;
    }

    // public double getAbsPosition() {
    //     return absoluteEncoder.getPosition().getValueAsDouble();
    // }
    // public double getAbsAbsPosition() {
    //     return absoluteEncoder.getAbsolutePosition().getValueAsDouble();
    // }

    /**
     * Retrieves the position of the turn motor
     * @return Position of the turn motor (radians)
     */
    public double getTurnPosition() {
        return turnEncoder.getPosition() * DriveConstants.turnEncoderResolution;
    }

    /**
     * Retrieves the velocity of the drive motor
     * @return Velocity of the drive motor (m/s)
     */
    public double getDriveVelocity() {
        return driveEncoder.getVelocity() * DriveConstants.driveSpeedResolution;
    }

    /**
     * Retrieves the velocity of the turn motor
     * @return Velocity of the turn motor (rad/s)
     */
    public double getTurnVelocity() {
        return turnEncoder.getVelocity() * DriveConstants.turnSpeedResolution;
    }

    public double getAbsVelocity() {
        return absoluteEncoder.getVelocity() * DriveConstants.turnSpeedResolution;
    }

    /**
     * Retrieves the position of the absolute encoder
     * @return Position of the absolute encoder (radians)
     */
    public double getAbsoluteEncoderRad() {
        double angle = absoluteEncoder.getPosition();
        angle *= -2 * Math.PI;
        angle += Math.PI;
        angle -= absoluteEncoderOffsetRad;
        if (absoluteEncoderReversed) {
            angle *= -1.0;
        }
        return angle;
    }

    /**
     * Resets the encoder values based on starting configuration
     * Drive acts as simple relative, resets to 0
     * Turn must match the current orientation, so resets to the absolute encoder's value
     */
    public void resetEncoders() {
        driveEncoder.setPosition(0.0);
        turnEncoder.setPosition(absoluteEncoder.getPosition());
        //SmartDashboard.putNumber("resetting turn encoder to", absoluteEncoder.getPosition());
    }

    public void configureFlexes() {
        
        SparkFlexConfig driveConfig = new SparkFlexConfig();
        driveConfig.signals.primaryEncoderVelocityPeriodMs(15);
        driveConfig.signals.primaryEncoderPositionPeriodMs(15);
        driveConfig.inverted(driveMotorReversed);
    
        
        SparkFlexConfig turnConfig = new SparkFlexConfig();
        turnConfig.signals.absoluteEncoderPositionPeriodMs(15);
        turnConfig.signals.primaryEncoderVelocityPeriodMs(15);
        turnConfig.signals.primaryEncoderPositionPeriodMs(15);
        turnConfig.inverted(turnMotorReversed);
        
        
        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    /**
     * Retrieves the current Swerve Module State
     * @return The current Swerve Module State
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getAbsoluteEncoderRad()));
    }

    public SwerveModulePosition getPosition() {
       return new SwerveModulePosition(getDrivePosition(), new Rotation2d(absoluteEncoder.getPosition()));
    }

    

    /**
     * Sets the motor speeds based on the current Swerve Module State
     * @param state
     */
    public void setDesiredState(SwerveModuleState state) {

        if (Math.abs(state.speedMetersPerSecond) < .001) { // Makes the motors stop with no joystick input instead of reset
            stop();
            return;
        }

     
       
        state.optimize(getState().angle);


        //if(this.driveMotorReversed) driveMotor.set((state.speedMetersPerSecond / DriveConstants.maxSpeed) * -1);
        //else 
        //driveMotor.set(state.speedMetersPerSecond / DriveConstants.maxSpeed);
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.maxSpeed);

        //if(this.turnMotorReversed) turnMotor.set(-1 * turnPID.calculate(getAbsoluteEncoderRad(), state.angle.getRadians()));
        //else 
          SmartDashboard.putNumber("desiredAngle", state.angle.getRadians());
           //SmartDashboard.putNumber("desiredBangle", getAbsoluteEncoderRad());
          SmartDashboard.putNumber("tpid", turnPID.calculate(getAbsoluteEncoderRad(), state.angle.getRadians()));
         
         
        turnMotor.set(turnPID.calculate(getAbsoluteEncoderRad(), state.angle.getRadians()));

    }

    /**
     * Stops both motors in the module
     */
    public void stop() {
        driveMotor.set(0);
        turnMotor.set(0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        
         //SmartDashboard.putNumber("spd", getState().speedMetersPerSecond);
         
        
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
