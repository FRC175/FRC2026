// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SwerveModule;

public class Swerve extends SubsystemBase {

    private final SwerveModule frontRight;
    private final SwerveModule frontLeft;
    private final SwerveModule backRight;
    private final SwerveModule backLeft;

    /** Creates a new Swerve System. */
    public Swerve() {

        frontRight = new SwerveModule(1, 2, 3, false, false, 0, false);
        frontLeft = new SwerveModule(4, 5, 6, false, false, 0, false);
        backRight = new SwerveModule(7, 8, 9, false, false, 0, false);
        backLeft = new SwerveModule(10, 11, 12, false, false, 0, false);

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
