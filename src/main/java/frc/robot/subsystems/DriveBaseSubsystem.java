/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.MotorController;
import frc.robot.utilities.PIDSpeedController;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.*;

import com.revrobotics.ControlType;

public class DriveBaseSubsystem extends SubsystemBase implements BiConsumer<Double, Double>, Supplier<Pose2d> {
  /**
   * Creates a new DriveBaseSubsystem.
   */

  private final Joystick mDriverJoystick;
  private final MotorController[] mMotorControllers = new MotorController[6];
  private final DifferentialDrive mDiffDrive;
  private boolean mIsArcadeDrive = true;
  private final DifferentialDriveOdometry mOdometry;
  private final Gyro mGyro = new ADXRS450_Gyro();
  private final DifferentialDriveKinematics mDifferentialDriveKinematics = new DifferentialDriveKinematics(Constants.kWheelBaseTrackWidthMeters);

  public DriveBaseSubsystem(Joystick joystick) {
    mDriverJoystick = joystick;    
    mMotorControllers[Constants.kDriveLeftFrontIndex] = new MotorController("Differential Left Front", Constants.kDriveLeftFront, Constants.kDriveBaseCurrentLimit, true);
    mMotorControllers[Constants.kDriveLeftMiddleIndex] = new MotorController("Differential Left Middle", Constants.kDriveLeftMiddle, Constants.kDriveBaseCurrentLimit);
    mMotorControllers[Constants.kDriveLeftRearIndex] = new MotorController("Differential Left Rear", Constants.kDriveLeftRear, Constants.kDriveBaseCurrentLimit);
    mMotorControllers[Constants.kDriveRightFrontIndex] = new MotorController("Differential Right Front", Constants.kDriveRightFront, Constants.kDriveBaseCurrentLimit, true);
    mMotorControllers[Constants.kDriveRightMiddleIndex] = new MotorController("Differential Right Middle", Constants.kDriveRightMiddle, Constants.kDriveBaseCurrentLimit);
    mMotorControllers[Constants.kDriveRightRearIndex] = new MotorController("Differential Right Rear", Constants.kDriveRightRear, Constants.kDriveBaseCurrentLimit);
    
    mMotorControllers[Constants.kDriveLeftRearIndex].getSparkMax().follow(mMotorControllers[Constants.kDriveLeftFrontIndex].getSparkMax());
    mMotorControllers[Constants.kDriveLeftMiddleIndex].getSparkMax().follow(mMotorControllers[Constants.kDriveLeftFrontIndex].getSparkMax());
    mMotorControllers[Constants.kDriveRightRearIndex].getSparkMax().follow(mMotorControllers[Constants.kDriveRightFrontIndex].getSparkMax());
    mMotorControllers[Constants.kDriveRightMiddleIndex].getSparkMax().follow(mMotorControllers[Constants.kDriveRightFrontIndex].getSparkMax());
    mDiffDrive = new DifferentialDrive(new PIDSpeedController(mMotorControllers[Constants.kDriveLeftFrontIndex], Constants.kSparkMaxMaximumRPM),
                                       new PIDSpeedController(mMotorControllers[Constants.kDriveRightFrontIndex], Constants.kSparkMaxMaximumRPM));

    mOdometry = new DifferentialDriveOdometry(mGyro.getRotation2d());
    mMotorControllers[Constants.kDriveLeftFrontIndex].getEncoder().setPositionConversionFactor(Constants.kPositionConversionFactor);
    mMotorControllers[Constants.kDriveRightFrontIndex].getEncoder().setPositionConversionFactor(Constants.kPositionConversionFactor);
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

  public Pose2d getPose() {
    return mOdometry.getPoseMeters();
  }

  public DifferentialDriveKinematics getDifferentialDriveKinematics() {
    return mDifferentialDriveKinematics;
  }

  @Override
  public void periodic()
  {
    for(int i = 0; i < mMotorControllers.length; i++) {
      mMotorControllers[i].updateSmartDashboard();
    }
    SmartDashboard.putBoolean("Arcade Drive", mIsArcadeDrive);
    mOdometry.update(mGyro.getRotation2d(), mMotorControllers[Constants.kDriveLeftFrontIndex].getEncoder().getPosition(), 
      mMotorControllers[Constants.kDriveRightFrontIndex].getEncoder().getPosition());
  }

  @Override
  public void accept(Double arg0, Double arg1) {
    //arg0 is left drivebase velocity in wheel mps, and arg1 is drivebase right velocity in wheel mps
    //We do arg divided by the conversionFactorConstant to convert wheel speed mps to motor rpm
    mMotorControllers[Constants.kDriveLeftFrontIndex].getPID().setReference(arg0 / Constants.kPositionConversionFactor, ControlType.kVelocity);
    mMotorControllers[Constants.kDriveRightFrontIndex].getPID().setReference(arg1 / Constants.kPositionConversionFactor, ControlType.kVelocity);
  }

  @Override
  public Pose2d get() {
    return getPose();
  }
}


