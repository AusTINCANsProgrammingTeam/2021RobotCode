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
  private double mP = 0.0;
  
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
    mShooterSubsystem.setLightStatus(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rotation = 0;
    if(mShooterSubsystem.isTargetInCameraFrame()) {
      double targetX = mShooterSubsystem.getTargetX();
      double desiredX = mShooterSubsystem.getDesiredTargetX();
      if(Math.abs(targetX - desiredX) > Constants.kLimelightDrivebaseTolerance) {
        rotation = mP * (targetX - desiredX);
      }
    }
    else {
      rotation = Constants.kTargetRotationSeekSpeed;
    }    
    mDriveBaseSubsystem.arcadeDrive(rotation);
    SmartDashboard.putNumber("Rotation Value", rotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // This command is intended to always be running while the other shooter commands are running,
  // so we return false because we'll let the ParallelRaceGroup cancel this command.
  @Override
  public boolean isFinished() {
    return false;
  }
}
