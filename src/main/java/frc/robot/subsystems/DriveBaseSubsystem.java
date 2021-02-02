/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.MotorController;

import com.revrobotics.CANSparkMax.ExternalFollower;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveBaseSubsystem extends SubsystemBase {
  /**
   * Creates a new DriveBaseSubsystem.
   */

  private final Joystick mDriverJoystick;
  private final MotorController[] mMotorControllers = new MotorController[6];
  private final DifferentialDrive mDiffDrive;
  private boolean mIsArcadeDrive = true;

  public DriveBaseSubsystem(Joystick joystick) {
    mDriverJoystick = joystick;    
    mMotorControllers[Constants.kDriveLeftFrontIndex] = new MotorController("Differential Left Front", Constants.kDriveLeftFront, Constants.kDriveBaseCurrentLimit);
    mMotorControllers[Constants.kDriveLeftMiddleIndex] = new MotorController("Differential Left Middle", Constants.kDriveLeftMiddle, Constants.kDriveBaseCurrentLimit);
    mMotorControllers[Constants.kDriveLeftRearIndex] = new MotorController("Differential Left Rear", Constants.kDriveLeftRear, Constants.kDriveBaseCurrentLimit);
    mMotorControllers[Constants.kDriveRightFrontIndex] = new MotorController("Differential Right Front", Constants.kDriveRightFront, Constants.kDriveBaseCurrentLimit);
    mMotorControllers[Constants.kDriveRightMiddleIndex] = new MotorController("Differential Right Middle", Constants.kDriveRightMiddle, Constants.kDriveBaseCurrentLimit);
    mMotorControllers[Constants.kDriveRightRearIndex] = new MotorController("Differential Right Rear", Constants.kDriveRightRear, Constants.kDriveBaseCurrentLimit);
    //The gear boxes are mirrored and one side needs to be inverted
    for(int i = Constants.kDriveLeftFrontIndex; i <= Constants.kDriveLeftRearIndex; i++) {
      mMotorControllers[i].getSparkMax().setInverted(true);
    }
    mMotorControllers[Constants.kDriveLeftRearIndex].getSparkMax().follow(mMotorControllers[Constants.kDriveLeftFrontIndex].getSparkMax());
    mMotorControllers[Constants.kDriveLeftMiddleIndex].getSparkMax().follow(mMotorControllers[Constants.kDriveLeftFrontIndex].getSparkMax());
    mMotorControllers[Constants.kDriveRightRearIndex].getSparkMax().follow(mMotorControllers[Constants.kDriveRightFrontIndex].getSparkMax());
    mMotorControllers[Constants.kDriveRightMiddleIndex].getSparkMax().follow(mMotorControllers[Constants.kDriveRightFrontIndex].getSparkMax());
    mDiffDrive = new DifferentialDrive(mMotorControllers[Constants.kDriveLeftFrontIndex].getSparkMax(), mMotorControllers[Constants.kDriveRightFrontIndex].getSparkMax());
  }

  public void arcadeDrive() {
    mDiffDrive.arcadeDrive(mDriverJoystick.getRawAxis(Constants.kLeftJoystickAxisY), mDriverJoystick.getRawAxis(Constants.kRightJoystickAxisX));
  }

  public void tankDrive() {
    mDiffDrive.tankDrive(mDriverJoystick.getRawAxis(Constants.kLeftJoystickAxisY), mDriverJoystick.getRawAxis(Constants.kRightJoystickAxisY));
  }

  public void drive() {
    if(mIsArcadeDrive)
      arcadeDrive();
    else
      tankDrive();
  }

  public void toggleDriveMode() {
    mIsArcadeDrive = !mIsArcadeDrive;
  }

  public boolean isArcadeDrive() {
    return mIsArcadeDrive;
  }

  public void stopMotors() {
    //We call .arcadeDrive with a speed and rotation of zero in hopes to command a zero output to the motors
    mDiffDrive.arcadeDrive(0.0, 0.0);
  }

  @Override
  public void periodic()
  {
    for(int i = 0; i < mMotorControllers.length; i++) {
      mMotorControllers[i].updateSmartDashboard();
    }
    SmartDashboard.putBoolean("Arcade Drive", mIsArcadeDrive);
  }
}


