/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.common.hardware.Limelight;
import frc.robot.subsystems.ShooterSubsystem;

public class SetupShooterCommand extends CommandBase {
  /**
   * Creates a new ShooterCommand.
   */

  private final ShooterSubsystem mShooterSubsystem;
  private final Limelight mLimeLight;


  // Steps to shooting
  // This assumes the vision target has been found. 
  // Determine if hood needs to be extended and do so if needed
  // Set the flywheel velocity based on range from goal
  // If flywheel velocity is within range and drivebase is aligned end setup command
  
  public SetupShooterCommand(ShooterSubsystem shooterSubsystem, Limelight limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem);
    mShooterSubsystem = shooterSubsystem;
    mLimeLight = limelight;
  } 

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(mLimeLight.isVisionTargetInCameraFrame()) {
      mShooterSubsystem.setHoodExtended(mShooterSubsystem.getHoodExtendRequired());
      mShooterSubsystem.setVelocity(mShooterSubsystem.getRequiredVelocityForDistance());
    }
    else{
      mShooterSubsystem.setVelocity(Constants.kWarmingShootupSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mShooterSubsystem.isReadyToShoot();
  }
}
