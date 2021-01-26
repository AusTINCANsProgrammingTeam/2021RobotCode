/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.Robot;
import frc.robot.CANSparkMaxWrap;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.*;

public class DriveBaseSubsystem extends SubsystemBase {
  /**
   * Creates a new DriveBaseSubsystem.
   */

  private final Joystick mDriverJoystick;
  private final CANSparkMax mRightFront;
  private final CANSparkMax mRightMiddle;
  private final CANSparkMax mRightRear;
  private final CANSparkMax mLeftFront;
  private final CANSparkMax mLeftMiddle;
  private final CANSparkMax mLeftRear;
  private final DifferentialDrive mDiffDrive;
  private final CANEncoder mRightFrontEncoder;
  private final CANEncoder mRightMiddleEncoder;
  private final CANEncoder mRightRearEncoder;
  private final CANEncoder mLeftFrontEncoder;
  private final CANEncoder mLeftMiddleEncoder;
  private final CANEncoder mLeftRearEncoder;
  private boolean mIsArcadeDrive = true;

  public DriveBaseSubsystem(Joystick joystick) {
    mDriverJoystick = joystick;
    mRightFront = new CANSparkMaxWrap(Constants.kDriveRightFront, MotorType.kBrushless);
    mRightMiddle = new CANSparkMaxWrap(Constants.kDriveRightMiddle, MotorType.kBrushless);
    mRightRear = new CANSparkMaxWrap(Constants.kDriveRightRear, MotorType.kBrushless);
    mLeftFront = new CANSparkMaxWrap(Constants.kDriveLeftFront, MotorType.kBrushless);
    mLeftMiddle = new CANSparkMaxWrap(Constants.kDriveLeftMiddle, MotorType.kBrushless);
    mLeftRear = new CANSparkMaxWrap(Constants.kDriveLeftRear, MotorType.kBrushless);
    //The gear boxes are mirrored and one side needs to be inverted
    mLeftFront.setInverted(true);
    mLeftMiddle.setInverted(true);
    mLeftRear.setInverted(true);
    mLeftRear.follow(mLeftFront);
    mLeftMiddle.follow(mLeftFront);
    mRightRear.follow(mRightFront);
    mRightMiddle.follow(mRightFront);
    //Reset the flash memory on the spark maxes so any saved configuration values do not carry over
    //We are configuring the spark maxes below
    mRightFront.restoreFactoryDefaults();
    mRightMiddle.restoreFactoryDefaults();
    mRightRear.restoreFactoryDefaults();
    mLeftFront.restoreFactoryDefaults();
    mLeftMiddle.restoreFactoryDefaults();
    mLeftRear.restoreFactoryDefaults();
    //Current limiting is required to prevent brown outs
    mRightFront.setSecondaryCurrentLimit(Constants.kDriveBaseCurrentLimit);
    mRightMiddle.setSecondaryCurrentLimit(Constants.kDriveBaseCurrentLimit);
    mRightRear.setSecondaryCurrentLimit(Constants.kDriveBaseCurrentLimit);
    mLeftFront.setSecondaryCurrentLimit(Constants.kDriveBaseCurrentLimit);
    mLeftMiddle.setSecondaryCurrentLimit(Constants.kDriveBaseCurrentLimit);
    mLeftRear.setSecondaryCurrentLimit(Constants.kDriveBaseCurrentLimit);
    mRightFrontEncoder = mRightFront.getEncoder();
    mRightMiddleEncoder = mRightMiddle.getEncoder();
    mRightRearEncoder = mRightRear.getEncoder();
    mLeftFrontEncoder = mLeftFront.getEncoder();
    mLeftMiddleEncoder = mLeftMiddle.getEncoder();
    mLeftRearEncoder = mLeftRear.getEncoder();
    mDiffDrive = new DifferentialDrive(mLeftFront, mRightFront);
  }

  public void arcadeDrive() {
    mDiffDrive.arcadeDrive(mDriverJoystick.getRawAxis(Constants.kLeftJoystick_yAxis), mDriverJoystick.getRawAxis(Constants.kRightJoystick_xAxis));
  }

  public void tankDrive() {
    mDiffDrive.tankDrive(mDriverJoystick.getRawAxis(Constants.kLeftJoystick_yAxis), mDriverJoystick.getRawAxis(Constants.kRightJoystick_yAxis));
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
    //The simulation crashes whenever .getEncoder() is called
    if (!Robot.isSimulation())
    {
      SmartDashboard.putNumber("Differential Right Front Encoder Position", mRightFrontEncoder.getPosition());
      SmartDashboard.putNumber("Differential Right Middle Encoder Position", mRightMiddleEncoder.getPosition());
      SmartDashboard.putNumber("Differential Right Rear Encoder Position", mRightRearEncoder.getPosition());
      SmartDashboard.putNumber("Differential Left Front Encoder Position", mLeftFrontEncoder.getPosition());
      SmartDashboard.putNumber("Differential Left Middle Encoder Position", mLeftMiddleEncoder.getPosition());
      SmartDashboard.putNumber("Differential Left Rear Encoder Position", mLeftRearEncoder.getPosition());
      SmartDashboard.putNumber("Differential Right Front Encoder Velocity", mRightFrontEncoder.getVelocity());
      SmartDashboard.putNumber("Differential Right Middle Encoder Velocity", mRightMiddleEncoder.getVelocity());
      SmartDashboard.putNumber("Differential Right Rear Encoder Velocity", mRightRearEncoder.getVelocity());
      SmartDashboard.putNumber("Differential Left Front Encoder Velocity", mLeftFrontEncoder.getVelocity());
      SmartDashboard.putNumber("Differential Left Middle Encoder Velocity", mLeftMiddleEncoder.getVelocity());
      SmartDashboard.putNumber("Differential Left Rear Encoder Velocity", mLeftRearEncoder.getVelocity());
    }
    SmartDashboard.putBoolean("Arcade Drive", mIsArcadeDrive);
    
  }  

}


