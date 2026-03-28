package frc.robot.subsystems.Drive;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkMaxAlternateEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.AbsoluteEncoder;

import com.ctre.phoenix6.hardware.CANcoder;

import frc.robot.utils.Vector;


/**
 * A class which represents a drive-turn motor combination, used for "Swerve Drive".
 */
public class AltSwerveModule {

    private final AbsoluteEncoder encoder;
    private final SparkFlex drive, turn;
    private final RelativeEncoder driveEncoder;
    private final double turnAngle;
    private double goalAngle;
    private static int id;
    private final int num;

    private boolean drivereversed, turnReversed;
    private boolean reverseDistance;

    public static double[] vectorMagnitudes = new double[4]; //FR,FL,BR,BL
    private static double[] vectorRotations = new double[4];
    
    /**
     * A Swerve Module is an abstraction of each individual turn-drive motor combination. They must work
     * in conjunction in order to accurately produce swerve motion.
     * @param drive CAN ID of the Drive motor.
     * @param turn CAN ID of the Turn motor.
     * @param encoder CAN ID of the specified CANCoder.
     * @param baseAngle The angle of maximum turning for the specified position.
     * @param basePosition A value to shift the CANCoders frame of reference, sets that position to be "0 degrees".
     */
    public AltSwerveModule(int drive, int turn,  double baseAngle, boolean drivereversed, boolean  turnReversed) {
        
        this.drive = new SparkFlex(drive, MotorType.kBrushless);
        this.turn = new SparkFlex(turn, MotorType.kBrushless);
        this.driveEncoder = this.drive.getEncoder();
        this.encoder = this.turn.getAbsoluteEncoder();
        this.turnAngle = baseAngle;
        this.num = id++;
        this.goalAngle = 0;
        this.drivereversed = drivereversed;
         this.turnReversed = turnReversed;
        this.reverseDistance = false;

        this.driveEncoder.setPosition(0);
    }

    /**
     * Finds the goal vector with the largest magnitude.
     * @return the magnitude of the vector
     * 
     */
    private static double maxMag() {
        return Math.max(Math.max(vectorMagnitudes[0], vectorMagnitudes[1]), Math.max(vectorMagnitudes[2], vectorMagnitudes[3]));
    }


    /**
     * Runs the drive motors as a specified power.
     * @param power : [0,1]
     */
    public void driveOpenLoop(double power) {
        drive.set(power);
    }

    /**
     * Runs the turn motors at a specified power.
     * @param power : [0,1]
     */
    public void turnOpenLoop(double power) {
        turn.set(power);
    }

    /**
     * Finds the outputs for the turn and drive motors. <p>Passes the data into the {@link #vectorMagnitudes}, and {@link #vectorRotations} static arrays. <p>Used in {@link #setOutputs()}.
     * @param transversal The input transversal vector from the joystick. <strong>[0,1]</strong>, <strong>[0,360)</strong> 
     * @param twist The input twist value from the joystick. <strong>[0,1]</strong>
     * @param gyroAngle The current gyro heading. <strong>[0, 360)</strong>
     * 
     */
    public Vector findControlledRotationVector(double twist) {
        Vector rotation = new Vector(twist * Math.cos(Math.toRadians(turnAngle)), twist * Math.sin(Math.toRadians(turnAngle))); // both vectors have angles with respect to x
        return rotation;
    }


    public Vector findLockedRotationVector(double lockAngle, double gyroAngle) {
        double delta = Math.abs(lockAngle-gyroAngle) < 180 ? (lockAngle-gyroAngle) : (lockAngle-gyroAngle) - 360 * Math.signum(lockAngle-gyroAngle);
        double twist = (delta / 180.0) * 1.0;
        Vector rotation = new Vector(twist * Math.cos(Math.toRadians(turnAngle)), twist * Math.sin(Math.toRadians(turnAngle))); // both vectors have angles with respect to x
        return rotation;
    }
    
    public void calculateRawOutputs(Vector transversal, Vector rotation) {
        // both vectors have angles with respect to x
        Vector goal = transversal.add(rotation);

        vectorMagnitudes[num] = goal.getMagnitude();

        goalAngle = goal.getAngle();

        if (Math.abs(goal.getMagnitude()) > 0.01) goalAngle = goal.getAngle();

        double delta = findDelta(goalAngle);

        if (Math.abs(delta) > 90) {
            goalAngle = (goalAngle + 180) % 360;
            drivereversed = true;
        } else { 
            drivereversed = false;
        }

        delta = findDelta(goalAngle);

        double maxSpeed = 1.0;
        double turnOut = Math.abs(delta) > 1 ? ((delta / 180) * maxSpeed) : 0;
        turnOut *= -1.0;
        vectorRotations[num] = turnOut;

        // SmartDashboard.putNumber("Goal Angle", goalAngle);
    }

    /**
     * Sets the outputs for the drive and turn motors. <p>{@link #calculateRawOutputs()} {@link #calculateRawLockOutputs()}
     */
    public void setOutputs() {
        double maxOutput = maxMag();
        if (maxOutput > 1.0) {
            for (int i = 0; i < vectorMagnitudes.length; ++i) {
                vectorMagnitudes[i] /= maxOutput;
            }
        }
        if (drivereversed) drive.set(-1 * vectorMagnitudes[num]);
        else drive.set(vectorMagnitudes[num]);
         if (turnReversed) turn.set(-1 * vectorRotations[num]);
        else turn.set(vectorRotations[num]);
        
    }

    public void lock() {
        double goalAngle = (turnAngle + 90) % 360;
        double delta = findDelta(goalAngle);

        double turnOut = Math.abs(delta) > 1 ? ((delta / 180) * 1.0) : 0;
        turnOut *= -1.0;
        vectorRotations[num] = turnOut;
        for (int i = 0; i < vectorMagnitudes.length; i++) {
            vectorMagnitudes[i] = 0;
        }
    }

 
    public double findDelta(double goalAngle) {
        // double val = controller.calculate(encoderToAngle(encoder.getAbsolutePosition().getValue()), goalAngle-gyroAngle);
        double currentAngle = getAngle();
        double delta = Math.abs(goalAngle-currentAngle) < 180 ? (goalAngle-currentAngle) : (goalAngle-currentAngle) - 360 * Math.signum(goalAngle-currentAngle);
        return delta;
    }

    public double encoderToAngle(double encoderVal) {
        return (((encoderVal * 360)  % 360) + 360) % 360;
    }

    public double getDriveEncoder() {
        return driveEncoder.getPosition();
    }

    public void resetDriveEncoder() {
        driveEncoder.setPosition(0);
    }

    public double getDriveDistance() {
        
        // return ((!reverseDistance && reversed) || (!reversed && reverseDistance)) ? (getDriveEncoder() / 6.75) * 0.319 : -1 * (getDriveEncoder() / 6.75) * 0.319;
        return getDriveEncoder() / 6.75 * 12.566;
    }

    public double getAngle() {
        return encoderToAngle(getShiftedEncoder());
    }

    public double getOdometryAngle() {
        if (getAngle() > 180) {
            return -(360-getAngle());
        } else {
            return getAngle();
        }
    }

    public double getEncoder() {
        return encoder.getPosition();
    }

    //  public double getAbsEncoder() {
    //     return encoder.getAbsolutePosition();
    // }
    public double getShiftedEncoder() {
        return getEncoder();
    }

    public boolean isReversed() {
        return drivereversed;
    }

    // Returns meters per second of module
    public double getVelocity() {
    
        return (0.4728678/encoder.getVelocity());
    }

    public void configureSparks(SparkFlexConfig config, ResetMode reset, PersistMode persist) {
        drive.configure(config, reset, persist);
        turn.configure(config, reset, persist);
    }
}