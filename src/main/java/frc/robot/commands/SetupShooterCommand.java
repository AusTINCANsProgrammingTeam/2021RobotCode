/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class SetupShooterCommand extends CommandBase {
  /**
   * Creates a new ShooterCommand.
   */

  private final ShooterSubsystem mShooterSubsystem;

  // Steps to shooting
  // This assumes the vision target has been found. 
  // Determine if hood needs to be extended and do so if needed
  // Compare the distance to allowed distance range to see if we are allowed to shoot from where we are currently at
  // Start aligning drivebase to goal
  // Set the flywheel velocity based on range from goal
  // If flywheel velocity is within range and drivebase is aligned end setup command
  
  private boolean mCanShootFromCurrentDistance;


  public SetupShooterCommand(ShooterSubsystem instance) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(instance);
    mShooterSubsystem = instance;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mShooterSubsystem.setHoodExtended(mShooterSubsystem.getHoodExtendRequired());
    mCanShootFromCurrentDistance = mShooterSubsystem.getRobotDistanceWithinRange();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mShooterSubsystem.setVelocity(mShooterSubsystem.getRequiredVelocityForDistance());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mShooterSubsystem.setHoodExtended(false);
    //Todo change this from commanding a speed of zero to letting it coast gently
    mShooterSubsystem.setVelocity(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}