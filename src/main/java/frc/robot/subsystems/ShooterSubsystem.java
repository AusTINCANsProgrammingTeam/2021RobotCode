/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CANSparkMaxWrap;
import frc.robot.Constants;
import frc.robot.Game;
import frc.robot.Robot;

public class ShooterSubsystem extends SubsystemBase {
  /**
   * Creates a new ShooterSubsystem.
   */
  private final CANSparkMax mShooterMotor;
  private final DoubleSolenoid mHoodDoubleSolenoid;
  private final CANPIDController mShooterPID;
  private final CANEncoder mShooterEncoder;
  private double mShooterVelocitySetpoint = 0.0;
  
  private double mP = 0.0;
  private double mI = 0.0;
  private double mD = 0.0;

  private double mGreenShootingVelocity = 0.0;
  private double mYellowShootingVelocity = 0.0;
  private double mBlueShootingVelocity = 0.0;
  private double mRedShootingVelocity = 0.0;

  public ShooterSubsystem() {
    mShooterMotor = new CANSparkMaxWrap(Constants.kShooterMotorPort, MotorType.kBrushless);
    mShooterMotor.restoreFactoryDefaults();
    mShooterMotor.setSmartCurrentLimit(Constants.kShooterMotorCurrentLimit);
    mHoodDoubleSolenoid = new DoubleSolenoid(Constants.kHoodDoubleSolenoidForwardChannel, Constants.kHoodDoubleSolenoidReverseChannel);
    mShooterPID = mShooterMotor.getPIDController();
    mShooterPID.setP(mP);
    mShooterPID.setI(mI);
    mShooterPID.setD(mD);
    SmartDashboard.putNumber("Shooter P Value", mP);
    SmartDashboard.putNumber("Shooter I Value", mI);
    SmartDashboard.putNumber("Shooter D Value", mD);
    
    mShooterEncoder = mShooterMotor.getEncoder();
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
    mShooterPID.setReference(velocity, ControlType.kVelocity);
  }

  public boolean getMotorVelocityWithinRange() {
    // Compare if the motor velocity is within Constants.kShooterVelocityRange range
    return (Math.abs(mShooterVelocitySetpoint - mShooterEncoder.getVelocity()) < mShooterVelocitySetpoint * Constants.kShooterVelocityRange);
  }

  public boolean getRobotDistanceWithinRange() {
    return getDistanceFromGoal() < getDistanceLimit();
  }

  public double getDistanceFromGoal() {
    return SmartDashboard.getNumber("Distance from Goal", 0.0);
  }

  public double getAngleFromGoal() {
    return SmartDashboard.getNumber("Angle from Goal", 0.0);
  }

  public boolean getAngleAligned() {
    return getAngleFromGoal() == 0;
  }

  public boolean getReadyToShoot() {
    return (getMotorVelocityWithinRange() && getAngleAligned());
  }

  public double getRequiredVelocityForDistance() {
    double velocity; 
    if (Constants.kGreenShootingZone < getDistanceFromGoal()) {
      velocity = mGreenShootingVelocity;
    } 
    else if (Constants.kYellowShootingZone < getDistanceFromGoal()) {
      velocity = mYellowShootingVelocity;
    } 
    else if (Constants.kBlueShootingZone < getDistanceFromGoal()) {
      velocity = mBlueShootingVelocity;
    }
    else if (Constants.kRedShootingZone < getDistanceFromGoal()) {
      velocity = mRedShootingVelocity;
    }
    else {
      velocity = 0.0; // Fix
    }
    return velocity;
  }

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

    
    if (Robot.isReal()) {
      SmartDashboard.putNumber("Shooter Motor Actual Velocity", mShooterEncoder.getVelocity());
    }
    
  }
}
