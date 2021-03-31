/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

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

public class ShooterSubsystem extends SubsystemBase {
  /**
   * Creates a new ShooterSubsystem.
   */
  private final MotorController mShooterMotor;
  private final DoubleSolenoid mHoodDoubleSolenoid;
  private final CANPIDController mShooterPID;
  private double mShooterVelocitySetpoint = 0.0;
  
  private double mP = 0.25;
  private double mI = 0.0;
  private double mD = 0.0;

  private double mGreenShootingVelocity = 3800.0;
  private double mYellowShootingVelocity = 4100.0;
  private double mBlueShootingVelocity = 4400.0;
  private double mRedShootingVelocity = 4700.0;

  private double mDesiredTargetX = Constants.kShooterDesiredTargetLocation;

  private NetworkTable mLimelightTable;


  public ShooterSubsystem() {
    mShooterMotor = new MotorController("Shooter Motor", Constants.kShooterMotorPort, Constants.kShooterMotorCurrentLimit);
    mHoodDoubleSolenoid = new DoubleSolenoid(Constants.kHoodDoubleSolenoidForwardChannel, Constants.kHoodDoubleSolenoidReverseChannel);
    mShooterPID = mShooterMotor.getSparkMax().getPIDController();
    mShooterPID.setP(mP);
    mShooterPID.setI(mI);
    mShooterPID.setD(mD);
    SmartDashboard.putNumber("Shooter P Value", mP);
    SmartDashboard.putNumber("Shooter I Value", mI);
    SmartDashboard.putNumber("Shooter D Value", mD);
    mLimelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    setLightStatus(false);
  }

  // Value.kReverse is when the hood is extended
  public void setHoodExtended(boolean isExtended) {
    if (isExtended)
      mHoodDoubleSolenoid.set(Value.kReverse);
    else  
      mHoodDoubleSolenoid.set(Value.kForward);
  }
  
  public boolean getHoodExtendRequired() {
    return getDistanceFromGoal() > Constants.kHoodExtendRequiredDistance;
  }

  public void setVelocity(double velocity) {
    mShooterVelocitySetpoint = velocity;
    if(velocity == 0.0) {
      mShooterPID.setReference(0.0, ControlType.kVoltage);
    }
    else{
      mShooterPID.setReference(velocity, ControlType.kVelocity);
    }
  }

  public boolean isMotorVelocityWithinRange() {
    // Compare if the motor velocity is within Constants.kShooterVelocityRange range
    return (Math.abs(mShooterVelocitySetpoint - mShooterMotor.getEncoder().getVelocity()) < mShooterVelocitySetpoint * Constants.kShooterVelocityPlusMinusPercent);
  }

  public boolean isRobotDistanceWithinRange() {
    return getDistanceFromGoal() < getDistanceLimit();
  }

  public double getDistanceFromGoal() {
    // distanceFromGoal is the following formula: (targetHeight - limelightHeight) / tan(limelightMountingAngle + limelightAngleToTarget)
    return (Constants.kTargetHeight - Constants.kLimelightHeight) / 
      Math.tan(Math.toRadians(Constants.kLimelightMountingAngle + mLimelightTable.getEntry("ty").getDouble(0.0))); 
  }
 
  public double getTargetX() {
    return mLimelightTable.getEntry("tx").getDouble(0.0);
  }

  public double getDesiredTargetX() {
    return mDesiredTargetX;
  }

  public boolean isTargetXAligned() {
    return Math.abs(getTargetX() - getDesiredTargetX()) > Constants.kLimelightDrivebaseTolerance;
  }

  public void setLightStatus(boolean isOn) {
    if (isOn) {
      mLimelightTable.getEntry("ledMode").setNumber(Constants.kLedOn);
    }
    else {
      mLimelightTable.getEntry("ledMode").setNumber(Constants.kLedOff);
    }
  }

  public boolean isTargetInCameraFrame() {
    return mLimelightTable.getEntry("tv").getDouble(0.0) > 0.0;
  }

  //Robot is close enough, shooter velocity is close enough, angle is close enough, target is on camera
  public boolean isReadyToShoot() {    
    return (isTargetInCameraFrame() && isMotorVelocityWithinRange() && isTargetXAligned() && isRobotDistanceWithinRange());
  }

  public double getRequiredVelocityForDistance() {
    double velocity; 
    if (getDistanceFromGoal() < Constants.kGreenShootingZone) {
      velocity = mGreenShootingVelocity;
    } 
    else if (getDistanceFromGoal() < Constants.kYellowShootingZone) {
      velocity = mYellowShootingVelocity;
    } 
    else if (getDistanceFromGoal() < Constants.kBlueShootingZone) {
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

  private void updatePIDFromSmartDashboard()
  {
    if (SmartDashboard.getNumber("Shooter P Value", mP) != mP) {
      mP = SmartDashboard.getNumber("Shooter P Value", mP);
      mShooterPID.setP(mP);
    }
    if (SmartDashboard.getNumber("Shooter I Value", mI) != mI) {
      mI = SmartDashboard.getNumber("Shooter I Value", mI);
      mShooterPID.setI(mI);
    }
    if (SmartDashboard.getNumber("Shooter D Value", mD) != mD) {
      mD = SmartDashboard.getNumber("Shooter D Value", mD);
      mShooterPID.setD(mD);
    }
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Motor Velocity Setpoint", mShooterVelocitySetpoint);
    updatePIDFromSmartDashboard();
    updateZoneVelocityFromSmartDashboard();

    SmartDashboard.putBoolean("Hood Extended", mHoodDoubleSolenoid.get() == Value.kReverse);
    mShooterMotor.updateSmartDashboard();
    mDesiredTargetX = SmartDashboard.getNumber("Desired Target X", mDesiredTargetX);

    SmartDashboard.putBoolean("Target in Camera Frame", isTargetInCameraFrame());
    SmartDashboard.putBoolean("Motor Velocity Within Range", isMotorVelocityWithinRange());
    SmartDashboard.putBoolean("Target X Aligned", isTargetXAligned());
    SmartDashboard.putBoolean("Robot Distance Within Range", isRobotDistanceWithinRange());
    SmartDashboard.putNumber("Distance From Goal", getDistanceFromGoal());
    SmartDashboard.putNumber("Distance Limit", getDistanceLimit());
    SmartDashboard.putBoolean("Ready to Shoot", isReadyToShoot()); 
    SmartDashboard.putNumber("Required Velocity For Distance", getRequiredVelocityForDistance());
  }
}
