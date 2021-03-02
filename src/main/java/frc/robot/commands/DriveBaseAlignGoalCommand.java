/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveBaseSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class DriveBaseAlignGoalCommand extends CommandBase {
  /**
   * Creates a new DriveBaseAlignGoalCommand.
   */
  private final ShooterSubsystem mShooterSubsystem;
  private final DriveBaseSubsystem mDriveBaseSubsystem;
  private double mP = 1.0;
  
  public DriveBaseAlignGoalCommand(ShooterSubsystem shooterSubsystem, DriveBaseSubsystem driveBaseSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveBaseSubsystem);
    mShooterSubsystem = shooterSubsystem;
    mDriveBaseSubsystem = driveBaseSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mP = SmartDashboard.getNumber("DriveBase Align P", mP);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {    
    //Todo add call to drivebase to setAngle from mShooterSubsystem.getAngleFromGoal();
    double rotation;
    if(!mShooterSubsystem.isOnCamera()) {
      rotation = 1.0;
    }
    else {
      rotation = 0.0;
      double targetX = mShooterSubsystem.getTargetX();
      double desiredX = mShooterSubsystem.getDesiredTargetX();
      if(Math.abs(targetX - desiredX) > Constants.kLimelightDrivebaseTolerance) {
        rotation = mP * (targetX - desiredX);
      }
    }
    mDriveBaseSubsystem.arcadeDrive(rotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
