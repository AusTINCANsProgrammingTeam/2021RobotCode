// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.DriveBaseSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;

import java.io.*;
import java.nio.file.Path;

public class RunPathCommand extends CommandBase {
  /** Creates a new RunPathCommand. */
  
  private final DriveBaseSubsystem mDriveBaseSubsystem;
  private Trajectory mTrajectory = new Trajectory();
  private String mPathName;
  public RunPathCommand(DriveBaseSubsystem driveBaseSubsystem, String pathName) {
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveBaseSubsystem);
    mDriveBaseSubsystem = driveBaseSubsystem;
    mPathName = pathName;
    try {
      Path path = Filesystem.getDeployDirectory().toPath().resolve(mPathName);
      mTrajectory = TrajectoryUtil.fromPathweaverJson(path);
      
    } 
    catch(IOException ex) {
      DriverStation.reportError("Could not find file: " + mPathName, ex.getStackTrace());
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
