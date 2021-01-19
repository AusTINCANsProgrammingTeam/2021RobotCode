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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.*;

public class DriveBaseSubsystem extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */

  private final Joystick m_js;
  private final CANSparkMax[] mDrivebaseMotors = new CANSparkMax[4];
  private final DifferentialDrive diffDrive;
  private boolean isArcadeDrive = true;

  public DriveBaseSubsystem(Joystick js) {
    m_js = js;
    for (int i = 0; i < mDrivebaseMotors.length; i++) {
      mDrivebaseMotors[i] = new CANSparkMaxWrap(i, MotorType.kBrushless);
    }
    mDrivebaseMotors[Constants.kLeftFront].setInverted(true);
    mDrivebaseMotors[Constants.kLeftRear].setInverted(true);
    mDrivebaseMotors[Constants.kLeftRear].follow(mDrivebaseMotors[Constants.kLeftFront]);
    mDrivebaseMotors[Constants.kRightRear].follow(mDrivebaseMotors[Constants.kRightFront]);
    diffDrive = new DifferentialDrive(mDrivebaseMotors[Constants.kLeftFront], mDrivebaseMotors[Constants.kRightFront]);
  }

  public void arcadeDrive() {
    diffDrive.arcadeDrive(m_js.getRawAxis(Constants.kLeftJoystick_yAxis), m_js.getRawAxis(Constants.kRightJoystick_xAxis));
  }

  public void tankDrive() {
    diffDrive.tankDrive(m_js.getRawAxis(Constants.kLeftJoystick_yAxis), m_js.getRawAxis(Constants.kRightJoystick_yAxis));
  }

  public void drive() {
    if(isArcadeDrive)
      arcadeDrive();
    else
      tankDrive();
  }

  public void toggleDriveMode() {
    isArcadeDrive = !isArcadeDrive;
  }

  public boolean isArcadeDrive() {
    return isArcadeDrive;
  }

  public void stopMotors() {
    diffDrive.stopMotor();
  }

  @Override
  public void periodic() {
  }
  
}
