/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

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
   * Creates a new ExampleSubsystem.
   */

  private final Joystick mDriverJoystick;
  private final CANSparkMax mRightFront;
  private final CANSparkMax mRightRear;
  private final CANSparkMax mLeftFront;
  private final CANSparkMax mLeftRear;
  private final DifferentialDrive mDiffDrive;
  private boolean mIsArcadeDrive = true;

  public DriveBaseSubsystem(Joystick js) {
    mDriverJoystick = js;
    mRightFront = new CANSparkMaxWrap(Constants.kDriveRightFront, MotorType.kBrushless);
    mRightRear = new CANSparkMaxWrap(Constants.kDriveRightRear, MotorType.kBrushless);
    mLeftFront = new CANSparkMaxWrap(Constants.kDriveLeftFront, MotorType.kBrushless);
    mLeftRear = new CANSparkMaxWrap(Constants.kDriveLeftRear, MotorType.kBrushless);
    mLeftFront.setInverted(true);
    mLeftRear.setInverted(true);
    mLeftRear.follow(mLeftFront);
    mRightRear.follow(mRightFront);
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
    mDiffDrive.arcadeDrive(0.0, 0.0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Front Encoder Position", mLeftFront.getEncoder().getPosition());
    SmartDashboard.putNumber("Right Front Encoder Position", mRightFront.getEncoder().getPosition());
    SmartDashboard.putNumber("Left Front Encoder Velocity", mLeftFront.getEncoder().getVelocity());
    SmartDashboard.putNumber("Right Front Encoder Velocity", mRightFront.getEncoder().getVelocity());
    SmartDashboard.putBoolean("Arcade Drive", mIsArcadeDrive);

  }
  
}
