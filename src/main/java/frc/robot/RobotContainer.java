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
import frc.robot.common.hardware.Limelight;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.commands.DriveBaseTeleopCommand;
import frc.robot.commands.IntakeSpinMotorBackwardCommand;
import frc.robot.commands.IntakeSpinMotorForwardCommand;
import frc.robot.subsystems.DriveBaseSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.CycleHopperCommand;
import frc.robot.subsystems.HopperSubsystem;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private final Joystick mDriverJoystick = new Joystick(Constants.kJoystickPort);

  private final HopperSubsystem mHopperSubsystem = new HopperSubsystem();
  private final Limelight mLimelight = new Limelight(98.0, 21.0, 20.0, 0.3, 0.0);
  private final ShooterSubsystem mShooterSubsystem = new ShooterSubsystem(mLimelight);
  private final DriveBaseSubsystem mDriveBaseSubsystem = new DriveBaseSubsystem(mDriverJoystick);

  private JoystickButton[] mButtons = new JoystickButton[11]; //Buttons #1-10

  private final IntakeSubsystem mIntakeSubsystem = new IntakeSubsystem();
  private final IntakeSpinMotorForwardCommand mIntakeSpinMotorForwardCommand = new IntakeSpinMotorForwardCommand(mIntakeSubsystem);
  private final IntakeSpinMotorBackwardCommand mIntakeSpinMotorBackwardCommand = new IntakeSpinMotorBackwardCommand(mIntakeSubsystem);

  private final DriveBaseTeleopCommand mDefaultDriveCommand = new DriveBaseTeleopCommand(mDriveBaseSubsystem);  
  private final InstantCommand mSwitchDriveModeCommand = new InstantCommand(mDriveBaseSubsystem::toggleDriveMode, mDriveBaseSubsystem);
  private final CycleHopperCommand mCycleHopperCommand = new CycleHopperCommand(mHopperSubsystem);
  private final ShootCommandGroup mShootCommandGroup = new ShootCommandGroup(mShooterSubsystem, mDriveBaseSubsystem, mHopperSubsystem, mLimelight);
  private final StartEndCommand mIntakeExtendCommand = new StartEndCommand(mIntakeSubsystem::setIntakeExtended, mIntakeSubsystem::setIntakeRetracted);
  

  public RobotContainer() {
    // Configure the button bindings
    for (int i = 1; i < mButtons.length; i++) {
      mButtons[i] = new JoystickButton(mDriverJoystick, i);
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
    mButtons[Constants.kRightBumperButton].whenPressed(mSwitchDriveModeCommand);
    mButtons[Constants.kLeftBumperButton].whileHeld(mIntakeSpinMotorForwardCommand);
    mButtons[Constants.kXButton].whileHeld(mIntakeSpinMotorBackwardCommand);
    mButtons[Constants.kYButton].toggleWhenPressed(mIntakeExtendCommand);
    mButtons[Constants.kBButton].whileHeld(mShootCommandGroup);
    mButtons[Constants.kAButton].whenPressed(mSwitchDriveModeCommand);
    //The buttons below are just for testing functionality
    mButtons[Constants.kRightTriggerButton].whileHeld(mCycleHopperCommand);
  }


    //Below are bindings for testing functionality
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
