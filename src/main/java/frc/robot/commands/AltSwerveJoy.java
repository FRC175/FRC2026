package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Drive.AltSwerve;
import frc.robot.utils.Controller;
import frc.robot.utils.Utils;

public class AltSwerveJoy extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final AltSwerve drive;
    private final XboxController joy;

    public AltSwerveJoy(XboxController joy, AltSwerve drive) {
        this.joy = joy;
        this.drive = drive;

        addRequirements(drive);
    }

    @Override
    public void initialize() {

    }


    @Override
    public void execute() {
        double twistDeadband = 0.45;
        double twist = Utils.deadband(joy.getRightX(), twistDeadband) * -1;
        if (twist < 0) twist = -1 + (1/(1-twistDeadband)) * (twist + 1);
        else if (twist > 0) twist = 1 + (1/(1-twistDeadband)) * (twist - 1);
        drive.swerve(
            Utils.deadband(joy.getLeftX(), 0.1, 1 * (1)), 
            Utils.deadband(joy.getLeftY(), 0.1, 1 * (1)), 
            Math.pow((twist * 0.60), 1), 
            drive.getYaw());
        // drive.postEncoders();
        // drive.postYaw();
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}