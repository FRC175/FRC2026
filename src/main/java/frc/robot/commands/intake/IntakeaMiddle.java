// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.intake;

// import frc.robot.subsystems.Intake;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants.IntakeConstants;

// /** A command that deploys the Intake */
// public class IntakeaMiddle extends Command {
//   @SuppressWarnings("PMD.UnusedPrivateField")
//   private final Intake intake; 
//   private final PIDController stowController;
 
//   public IntakeaMiddle(Intake intake) {
//     this.intake = intake;
//     stowController = new PIDController(.5, 0, 0);
  
//     // Use addRequirements() here to declare subsystem dependencies.
//     addRequirements(intake);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     //intake.setDeployPosition(0);
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     if (intake.getAbsolutePosition() >= 100){
//       //deployed = true;
//       intake.setDeployVelocity(-.1);
//     } else if(intake.getAbsolutePosition() <= 45){
//       //deployed = false;
//       intake.setDeployVelocity(.1);
//     }
    
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     intake.isDeployed = !intake.isDeployed;
//     intake.setDeployVelocity(0);
    
//   }

//   // 300 is a dummy value.
//   @Override
//   public boolean isFinished() {
//    if (!deployed) {
//     if(intake.getDeployPosition() <= IntakeConstants.intakeMiddlePosition) {
//     return true;
//    } else return false;
//   } else {
//     if(deployed) {
//     return true;
//    } else return false;
//   }
//    //return (intake.getAbsolutePosition() == IntakeConstants.intakeMiddlePosition);
//   } 

// }
