/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.MotorController;
import frc.robot.Game;
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Game;
    
import frc.robot.MotorController;
import frc.robot.Robot;
import frc.robot.common.hardware.Limelight;

public class ShooterSubsystem extends SubsystemBase {
  /**
   * Creates a new ShooterSubsystem.
   */
  private final MotorController mShooterController;
  private final DoubleSolenoid mHoodDoubleSolenoid;
  private final Limelight mLimelight;
  private double mShooterVelocitySetpoint = 0.0;

  private double mGreenShootingVelocity = 3800.0;
  private double mYellowShootingVelocity = 4100.0;
  private double mBlueShootingVelocity = 4400.0;
  private double mRedShootingVelocity = 4700.0;

  public ShooterSubsystem() {
    mShooterController = new MotorController("Shooter", Constants.kShooterMotorPort, Constants.kShooterMotorCurrentLimit, true);
    mHoodDoubleSolenoid = new DoubleSolenoid(Constants.kHoodDoubleSolenoidForwardChannel, Constants.kHoodDoubleSolenoidReverseChannel);
    mLimelight = new Limelight(98.0, 21.0, 20.0, 0.3, 0.0);
    mLimelight.setLightStatus(false);
  }

  // Value.kReverse is when the hood is extended
  public void setHoodExtended(boolean isExtended) {
    if (isExtended)
      mHoodDoubleSolenoid.set(Value.kReverse);
    else  
      mHoodDoubleSolenoid.set(Value.kForward);
  }
  
  public boolean getHoodExtendRequired() {
    return mLimelight.getDistanceFromGoal() > Constants.kHoodExtendRequiredDistance;
  }

  public void setVelocity(double velocity) {
    mShooterVelocitySetpoint = velocity;
    if(velocity == 0.0) {
      mShooterController.getPID().setReference(0.0, ControlType.kVoltage);
    }
    else{
      mShooterController.getPID().setReference(velocity, ControlType.kVelocity);
    }
  }

  public boolean isMotorVelocityWithinRange() {
    // Compare if the motor velocity is within Constants.kShooterVelocityRange range
    return (Math.abs(mShooterVelocitySetpoint - mShooterController.getEncoder().getVelocity()) < mShooterVelocitySetpoint * Constants.kShooterVelocityPlusMinusPercent);
  }

  public boolean isRobotDistanceWithinRange() {
    return mLimelight.getDistanceFromGoal() < getDistanceLimit();
  }

  //Robot is close enough, shooter velocity is close enough, angle is close enough, target is on camera
  public boolean isReadyToShoot() {    
    return (isMotorVelocityWithinRange() && mLimelight.isTargetXAligned() && isRobotDistanceWithinRange());
  }

  public double getRequiredVelocityForDistance() {
    double velocity; 
    if (mLimelight.getDistanceFromGoal() < Constants.kGreenShootingZone) {
      velocity = mGreenShootingVelocity;
    } 
    else if (mLimelight.getDistanceFromGoal() < Constants.kYellowShootingZone) {
      velocity = mYellowShootingVelocity;
    } 
    else if (mLimelight.getDistanceFromGoal() < Constants.kBlueShootingZone) {
      velocity = mBlueShootingVelocity;
    }
    else{ //If the robot is past the blue zone, then we'll assume the robot is in the red zone
      velocity = mRedShootingVelocity;
    }
    return velocity;
  }

  //Only used for Power Port and Interstellar Accuracy for setting the max shooting range
  private double getDistanceLimit() {
    double limit;
    if (Game.getGame() == Game.GameType.PowerPort) {
      limit = Constants.kRedShootingZone;
    }
    else if (Game.getGame() == Game.GameType.InterstellarAccuracy) {
      limit = Constants.kBlueShootingZone;
    }
    else { 
      limit = 0;
    }
    return limit;
  }

  private void updateZoneVelocityFromSmartDashboard() {
    mGreenShootingVelocity = SmartDashboard.getNumber("Green Shooting Velocity", mGreenShootingVelocity);
    mYellowShootingVelocity = SmartDashboard.getNumber("Yellow Shooting Velocity", mYellowShootingVelocity);
    mBlueShootingVelocity = SmartDashboard.getNumber("Blue Shooting Velocity", mBlueShootingVelocity);
    mRedShootingVelocity = SmartDashboard.getNumber("Red Shooting Velocity", mRedShootingVelocity);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Motor Velocity Setpoint", mShooterVelocitySetpoint);
    updateZoneVelocityFromSmartDashboard();

    SmartDashboard.putBoolean("Hood Extended", mHoodDoubleSolenoid.get() == Value.kReverse);
    mShooterController.updateSmartDashboard();

    SmartDashboard.putBoolean("Motor Velocity Within Range", isMotorVelocityWithinRange());
    SmartDashboard.putBoolean("Robot Distance Within Range", isRobotDistanceWithinRange());
    SmartDashboard.putNumber("Distance Limit", getDistanceLimit());
    SmartDashboard.putBoolean("Ready to Shoot", isReadyToShoot()); 
    SmartDashboard.putNumber("Required Velocity For Distance", getRequiredVelocityForDistance());

    mLimelight.updateSmartDashboard();
  }
}
