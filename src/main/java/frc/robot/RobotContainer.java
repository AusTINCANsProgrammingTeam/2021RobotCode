/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ShootCommandGroup;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveBaseTeleopCommand;
import frc.robot.subsystems.DriveBaseSubsystem;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private final Joystick mDriverJoystick = new Joystick(Constants.kJoystickPort);

  private final ShooterSubsystem mShooterSubsystem = new ShooterSubsystem();
  private final DriveBaseSubsystem mDriveBaseSubsystem = new DriveBaseSubsystem(mDriverJoystick);

  private JoystickButton[] mButtons = new JoystickButton[10]; //Buttons #1-10

  private final DriveBaseTeleopCommand mDefaultDriveCommand = new DriveBaseTeleopCommand(mDriveBaseSubsystem);  
  private final InstantCommand mSwitchDriveModeCommand = new InstantCommand(mDriveBaseSubsystem::toggleDriveMode, mDriveBaseSubsystem);
  private final ShootCommandGroup mShootCommandGroup = new ShootCommandGroup(mShooterSubsystem);
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    for (int i = 0; i < mButtons.length; i++) {
      mButtons[i] = new JoystickButton(mDriverJoystick, i+1);
    }
    configureButtonBindings();
    mDriveBaseSubsystem.setDefaultCommand(mDefaultDriveCommand);
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  
  private void configureButtonBindings() {
    mButtons[Constants.kBButton].whileHeld(mShootCommandGroup);
    mButtons[Constants.kAButton].whenPressed(mSwitchDriveModeCommand);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
