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

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CANSparkMaxWrap;
import frc.robot.Constants;
import frc.robot.Robot;

public class ShooterSubsystem extends SubsystemBase {
  /**
   * Creates a new ShooterSubsystem.
   */
  private final CANSparkMax mShooterMotor;
  private final DoubleSolenoid mHoodDoubleSolenoid;
  private final CANPIDController mShooterPID;
  private double mShooterVelocitySetpoint = 0.0;
  private boolean isExtended = true;
  
  private double mP = 0.0;
  private double mI = 0.0;
  private double mD = 0.0;

  public ShooterSubsystem() {
    mShooterMotor = new CANSparkMaxWrap(Constants.kShooterMotorPort, MotorType.kBrushless);
    mHoodDoubleSolenoid = new DoubleSolenoid(Constants.kHoodDoubleSolenoidForwardChannel, Constants.kHoodDoubleSolenoidReverseChannel);
    mShooterPID = mShooterMotor.getPIDController();
    mShooterPID.setP(mP);
    mShooterPID.setI(mI);
    mShooterPID.setD(mD);
    SmartDashboard.putNumber("Shooter P Value", mP);
    SmartDashboard.putNumber("Shooter I Value", mI);
    SmartDashboard.putNumber("Shooter D Value", mD);
  }

  public void toggleHoodExtension() {
    isExtended = !isExtended;
    setHoodExtended(isExtended);
  }

  //Value.kReverse is when the hood is extended
  public void setHoodExtended(boolean isExtended) {
    if(isExtended)
      mHoodDoubleSolenoid.set(Value.kReverse);
    else  
      mHoodDoubleSolenoid.set(Value.kForward);
  }

  public void setVelocity(double velocity) {
    mShooterVelocitySetpoint = velocity;
    mShooterPID.setReference(velocity, ControlType.kVelocity);
  }

  private boolean compareDoubleSolenoidAndIsExtended() {
    boolean hoodExtended = (mHoodDoubleSolenoid.get()==Value.kReverse);
    return (isExtended == hoodExtended);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Motor Velocity Setpoint", mShooterVelocitySetpoint);
    if(SmartDashboard.getNumber("Shooter P Value", mP) != mP) {
      mP = SmartDashboard.getNumber("Shooter P Value", mP);
      mShooterPID.setP(mP);
    }
    if(SmartDashboard.getNumber("Shooter I Value", mI) != mI) {
      mI = SmartDashboard.getNumber("Shooter I Value", mI);
      mShooterPID.setI(mI);
    }
    if(SmartDashboard.getNumber("Shooter D Value", mD) != mD) {
      mD = SmartDashboard.getNumber("Shooter D Value", mD);
      mShooterPID.setD(mD);
    }
    SmartDashboard.putBoolean("Hood Extended", mHoodDoubleSolenoid.get() == Value.kReverse);
    if (Robot.isReal()) {
      SmartDashboard.putNumber("Shooter Motor Actual Velocity", mShooterMotor.getEncoder().getVelocity());
    }
    SmartDashboard.putBoolean("isExtended value", isExtended);
    SmartDashboard.putBoolean("isExtended and Double Solenoid aligned", compareDoubleSolenoidAndIsExtended()); //compareDoubleSolenoidAndIsExtended()
  }
}
