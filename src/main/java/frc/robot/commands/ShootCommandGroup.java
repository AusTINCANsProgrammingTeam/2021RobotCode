// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.subsystems.DriveBaseSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommandGroup extends SequentialCommandGroup {
  /** Creates a new ShootCommandGroup. */
  /*
        This command will end when either SetupShooterCommand is successful or if 
        the whole sequence is interrupted (e.g. when the button isn't being pressed)

        This command should align the drivebase to the goal, rev up the shooter flywheel motor, 
        set the hood to a certain position, and it should spin the spindexer. Lastly, once the command is
        interrupted or once it ends, ShooterEndCommand resets everything.

        SetupShooterCommand is the only command that can end the ParallelRaceGroup, as 
        DriveBaseAlignGoalCommand constantly needs to be running
  */
  public ShootCommandGroup(ShooterSubsystem shooterSubsystem, DriveBaseSubsystem driveBaseSubsystem) {
      addCommands(
        new ParallelCommandGroup(
          new DriveBaseAlignGoalCommand(shooterSubsystem, driveBaseSubsystem),
          new SequentialCommandGroup(
            new SetupShooterCommand(shooterSubsystem), 
            new CycleHopperCommand()
          ),
          new ShooterEndCommand(shooterSubsystem)
        )
      );
  }
}