// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANcoder;
import frc.robot.Constants.DriverConstants;

public class SwerveModule extends SubsystemBase {

    private final SparkFlex driveMotor;
    private final SparkFlex turnMotor;
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turnEncoder;

    private final CANcoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    private final PIDController turnPID;

    /** Creates a new SwerveModule. */
    public SwerveModule(int driveMoterID, int turnMotorID, int absoluteEncoderID,
            boolean driveMotorReversed,
            boolean turningMotorReversed, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        // Initialize motors
        driveMotor = new SparkFlex(driveMoterID, MotorType.kBrushless);
        turnMotor = new SparkFlex(turnMotorID, MotorType.kBrushless);

        // Set motor inversion if needed
        driveMotor.setInverted(driveMotorReversed); // TODO: Why that funky?
        turnMotor.setInverted(turningMotorReversed);

        // Initialize encoders
        driveEncoder = driveMotor.getEncoder();
        turnEncoder = turnMotor.getEncoder();

        // Initialize absolute encoder
        absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new CANcoder(absoluteEncoderID, new CANBus("CANivore_BUS"));

        // Initialize PID controller for turning motor (should only need P)
        turnPID = new PIDController(0.5, 0.0, 0.0);
        turnPID.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    // TODO: Add conversions

    /**
     * Retrieves the position of the drive motor
     * @return Position of the drive motor (UNITS)
     */
    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    /**
     * Retrieves the position of the turn motor
     * @return Position of the turn motor (radians)
     */
    public double getTurnPosition() {
        return turnEncoder.getPosition();
    }

    /**
     * Retrieves the velocity of the drive motor
     * @return Velocity of the drive motor (m/s)
     */
    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    /**
     * Retrieves the velocity of the turn motor
     * @return Velocity of the turn motor (rad/s)
     */
    public double getTurnVelocity() {
        return turnEncoder.getVelocity();
    }

    /**
     * Retrieves the position of the absolute encoder
     * @return Position of the absolute encoder (radians)
     */
    public double getAbsoluteEncoderRad() {
        double angle = absoluteEncoder.getPosition().getValueAsDouble();
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
        turnEncoder.setPosition(getAbsoluteEncoderRad());
    }

    /**
     * Retrieves the current Swerve Module State
     * @return The current Swerve Module State
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurnPosition()));
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

        driveMotor.set(state.speedMetersPerSecond / DriverConstants.maxSpeed);
        turnMotor.set(turnPID.calculate(getTurnPosition(), state.angle.getRadians()));

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
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
