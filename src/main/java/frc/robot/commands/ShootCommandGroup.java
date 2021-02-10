// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommandGroup extends SequentialCommandGroup {
  /** Creates a new ShootCommandGroup. */
  /*
        This command will end when either SetupShooterCommand is successful or if 
        the whole sequence is interrupted (e.g. when the button isn't being pressed)
  */
  public ShootCommandGroup(ShooterSubsystem shooterSubsystem) {
      addCommands(
        new ParallelRaceGroup(
          new DriveBaseAlignGoalCommand(shooterSubsystem),
          new SequentialCommandGroup(
            new SetupShooterCommand(shooterSubsystem),
            new CycleHopperCommand()
          ),
          new ShooterEndCommand(shooterSubsystem)
        )
      );
  }
}