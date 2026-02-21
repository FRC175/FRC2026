package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drive.Swerve;

public class SwerveJoystick extends Command {

    private final Swerve swerve;
    private final Supplier<Double> xSpeedFunction, ySpeedFunction, turnSpeedFunction;
    private final Supplier<Boolean> fieldOrientedFunction;
    private final SlewRateLimiter xLimiter, yLimiter, turnLimiter;

    public SwerveJoystick(Swerve swerve, Supplier<Double> xSpeedFunction, Supplier<Double> ySpeedFunction,
            Supplier<Double> turnSpeedFunction, Supplier<Boolean> fieldOrientedFunction) {
        //Swerve subsystem
        this.swerve = swerve;

        //Input functions
        this.xSpeedFunction = xSpeedFunction;
        this.ySpeedFunction = ySpeedFunction;
        this.turnSpeedFunction = turnSpeedFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;

        //Rate Limiters
        this.xLimiter = new SlewRateLimiter(DriveConstants.maxDriveAcceleration);
        this.yLimiter = new SlewRateLimiter(DriveConstants.maxDriveAcceleration);
        this.turnLimiter = new SlewRateLimiter(DriveConstants.maxAngularAcceleration);

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
        turnSpeed = Math.abs(turnSpeed) > .1 ? turnSpeed : 0.0;

        //Rate Limiter on joysticks and scale to 1/2 of max speed for Teleop
        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.maxTeleopSpeed;
        SmartDashboard.putNumber("Converted X Speed", xSpeed);
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.maxTeleopSpeed;
        SmartDashboard.putNumber("Converted Y Speed", ySpeed);
        turnSpeed = turnLimiter.calculate(turnSpeed) * (DriveConstants.maxSpeed * .5);
        SmartDashboard.putNumber("Converted Turn Speed", turnSpeed);

        //Create chassis speeds
        ChassisSpeeds chassisSpeeds;
        if(fieldOrientedFunction.get()) {
            //Field Relative Control
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turnSpeed, swerve.getRotation2d());
        } else {
            //Robot Relative Control
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turnSpeed);
        }

       // ChassisSpeeds.discretize(chassisSpeeds, .02);

        //Convert to array of module states
        SwerveModuleState[] moduleStates = DriveConstants.kinematics.toSwerveModuleStates(chassisSpeeds);
           
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
