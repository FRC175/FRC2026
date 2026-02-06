package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class SwerveJoystick extends Command {

    private final Swerve swerve;
    private final Supplier<Double> xSpeedFunction, ySpeedFunction, turnSpeedFunction;
    private final Supplier<Boolean> fieldOrientedFunction;


    public SwerveJoystick( Swerve swerve, Supplier<Double> xSpeedFunction, Supplier<Double> ySpeedFunction, Supplier<Double> turnSpeedFunction, Supplier<Boolean> fieldOrientedFunction) {
         this.swerve = swerve;
         this.xSpeedFunction = xSpeedFunction;
         this.ySpeedFunction =  ySpeedFunction;
         this.turnSpeedFunction = turnSpeedFunction;
         this.fieldOrientedFunction = fieldOrientedFunction;


    }
    
}
