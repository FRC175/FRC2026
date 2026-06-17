package frc.robot.commands.drive;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Drive.Swerve;

public class SwerveJoystick extends Command {

    private final Swerve swerve;
    private final Limelight limelight;
    private final Shooter shooter;
    private final Supplier<Double> xSpeedFunction, ySpeedFunction, turnSpeedFunction;
    private final Supplier<Boolean> fieldOrientedFunction, aimLockOn, strafeLockOn, autoAiming, lockPose;
    private final SlewRateLimiter xLimiter, yLimiter, turnLimiter;
    private final PIDController aimController, xController;
    private double currentAngle;

    public SwerveJoystick(Swerve swerve, Limelight limelight, Shooter shooter, Supplier<Double> xSpeedFunction, Supplier<Double> ySpeedFunction,
            Supplier<Double> turnSpeedFunction, Supplier<Boolean> fieldOrientedFunction, Supplier<Boolean> aimLockOn, Supplier<Boolean> autoAiming, Supplier<Boolean> strafeLockOn, Supplier<Boolean> lockPose) {
        //Swerve subsystem
        this.swerve = swerve;
        this.limelight = limelight;
        this.shooter = shooter;

        //Input functions
        this.xSpeedFunction = xSpeedFunction;
        this.ySpeedFunction = ySpeedFunction;
        this.turnSpeedFunction = turnSpeedFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.aimLockOn = aimLockOn;
        this.strafeLockOn = strafeLockOn;
        this.autoAiming = autoAiming;
        this.lockPose = lockPose;

        //Rate Limiters
        this.xLimiter = new SlewRateLimiter(DriveConstants.maxDriveAcceleration);
        this.yLimiter = new SlewRateLimiter(DriveConstants.maxDriveAcceleration);
        this.turnLimiter = new SlewRateLimiter(DriveConstants.maxAngularAcceleration);

        //PID for shooter aiming lock-on
        aimController = new PIDController(2,0,0);
        aimController.setTolerance(.01);

        //PID for Strafing
        xController = new PIDController(DriveConstants.pDriveConstants, 0, 0);
        xController.setTolerance(20);
       


        addRequirements(swerve);
        
    }

    @Override
    public void execute() {
        //Gets the joystick inputs
        double xSpeed = xSpeedFunction.get();
        SmartDashboard.putNumber("x-Stick", xSpeed);
        double ySpeed = ySpeedFunction.get();
         SmartDashboard.putNumber("y-Stick", ySpeed);
        double turnSpeed = turnSpeedFunction.get();
         SmartDashboard.putNumber("t-Stick", turnSpeed);

        //Apply the Deadband
        xSpeed = Math.abs(xSpeed) > .1 ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > .1 ? ySpeed : 0.0;
        turnSpeed = Math.abs(turnSpeed) > .15 ? turnSpeed : 0.0;

        //Rate Limiter on joysticks and scale to 75% of max speed for Teleop
        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.maxTeleopSpeed;
        SmartDashboard.putNumber("Converted X Speed", xSpeed);
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.maxTeleopSpeed;
        SmartDashboard.putNumber("Converted Y Speed", ySpeed);
        turnSpeed = turnLimiter.calculate(turnSpeed) * 1.2* Math.PI ;

        
        currentAngle = limelight.getTx();
        double effort = aimController.calculate(currentAngle, -.1);
 
        if(aimLockOn.get() || autoAiming.get()) {
            if (aimLockOn.get() && autoAiming.get()) {
                swerve.setAutoAiming(false);
            }
            turnSpeed = -effort*Math.PI;
            SmartDashboard.putNumber("Aim Turn Effort", effort);
            if (strafeLockOn.get()) {
                double xEffort = MathUtil.clamp(xController.calculate(shooter.getHeading(), 0), -1, 1) * DriveConstants.maxTeleopSpeed;

            }
            if (lockPose.get()) {{
                
            }} else {
               
            }
        } else SmartDashboard.putNumber("Converted Turn Speed", turnSpeed);

        //Create chassis speeds
        ChassisSpeeds chassisSpeeds;
        if(fieldOrientedFunction.get()) {
            //Field Relative Control
            
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turnSpeed, swerve.getRotation2d());
            swerve.chassisSpeeds = chassisSpeeds;
        } else {
            //Robot Relative Control
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turnSpeed);
            swerve.chassisSpeeds = chassisSpeeds;
        }

        //ChassisSpeeds.discretize(chassisSpeeds, .02);

        //Convert to array of module states
        SwerveModuleState[] moduleStates = {null, null, null, null};
            moduleStates = DriveConstants.kinematics.toSwerveModuleStates(chassisSpeeds);
           
        //SmartDashboard.putNumber("desiredWangle", moduleStates[3].angle.getRadians());
          
       
        //Send states to modules
        swerve.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
