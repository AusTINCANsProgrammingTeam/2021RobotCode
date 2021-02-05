/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.MotorController;

public class HopperSubsystem extends SubsystemBase {
  /**
   * Creates a new HopperSubsystem.
   */
  //2 motors - spindexer (spin in circle), bring up < limiting speed; color sensor (?)
  private final MotorController mSpindexerController;
  private final MotorController mPickupController;

  private final CANPIDController mSpindexerPID;
  private double mSpindexerP = 0.0;
  private double mSpindexerI = 0.0;
  private double mSpindexerD = 0.0;

  private final CANPIDController mPickupPID;
  private double mPickupP = 0.0;
  private double mPickupI = 0.0;
  private double mPickupD = 0.0;

  private double mSpindexerPositionSetpoint = 0.0;
  private double mPickupVelocitySetpoint = 0.0;

  public HopperSubsystem() { //possibility of needing PID (?)
    mSpindexerController = new MotorController("", Constants.kHopperSpindexerPort); //Todo: set name
    mSpindexerPID = mSpindexerController.getSparkMax().getPIDController();
    mSpindexerPID.setP(mSpindexerP);
    mSpindexerPID.setI(mSpindexerI);
    mSpindexerPID.setD(mSpindexerD);
    //Convert position value from NEO rotations to spindexer rotations
    mSpindexerController.getEncoder().setPositionConversionFactor(1.0/Constants.kSpindexerGearRatio);

    SmartDashboard.putNumber("Spindexer P Value", mSpindexerP);
    SmartDashboard.putNumber("Spindexer I Value", mSpindexerI);
    SmartDashboard.putNumber("Spindexer D Value", mSpindexerD);
    mPickupController = new MotorController("", Constants.kHopperPickupMotorPort); //Todo: set name
    mPickupPID = mPickupController.getSparkMax().getPIDController();
    mPickupPID.setP(mPickupP);
    mPickupPID.setI(mPickupI);
    mPickupPID.setD(mPickupD);
    SmartDashboard.putNumber("Pickup P Value", mPickupP);
    SmartDashboard.putNumber("Pickup I Value", mPickupI);
    SmartDashboard.putNumber("Pickup D Value", mPickupD);

  }

  public void setSpindexerPosition(double position) {
    mSpindexerPositionSetpoint = position;
    mSpindexerPID.setReference(position, ControlType.kPosition);
  }

  public void incrementSpindexPosition(double increment) {
    setSpindexerPosition(mSpindexerPositionSetpoint + increment);
  }

  public void setPickupVelocity(double velocity) {
    mPickupVelocitySetpoint = velocity;
    mPickupPID.setReference(velocity, ControlType.kVelocity);
  }

  private void updatePIDFromSmartDashboard() {
    if (SmartDashboard.getNumber("Spindexer P Value", mSpindexerP) != mSpindexerP) {
      mSpindexerP = SmartDashboard.getNumber("Spindexer P Value", mSpindexerP);
      mSpindexerPID.setP(mSpindexerP);
    }
    if (SmartDashboard.getNumber("Spindexer I Value", mSpindexerI) != mSpindexerI) {
      mSpindexerI = SmartDashboard.getNumber("Spindexer I Value", mSpindexerI);
      mSpindexerPID.setP(mSpindexerI);
    }
    if (SmartDashboard.getNumber("Spindexer D Value", mSpindexerD) != mSpindexerD) {
      mSpindexerD = SmartDashboard.getNumber("Spindexer D Value", mSpindexerD);
      mSpindexerPID.setP(mSpindexerD);
    }

    if (SmartDashboard.getNumber("Pickup P Value", mPickupP) != mPickupP) {
      mPickupP = SmartDashboard.getNumber("Pickup P Value", mPickupP);
      mPickupPID.setP(mPickupP);
    }
    if (SmartDashboard.getNumber("Pickup I Value", mPickupI) != mPickupI) {
      mPickupI = SmartDashboard.getNumber("Pickup I Value", mPickupI);
      mPickupPID.setP(mPickupI);
    }
    if (SmartDashboard.getNumber("Pickup D Value", mPickupD) != mPickupD) {
      mPickupD = SmartDashboard.getNumber("Pickup D Value", mPickupD);
      mPickupPID.setP(mPickupD);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updatePIDFromSmartDashboard();
    SmartDashboard.putNumber("Spindexer Position Setpoint", mSpindexerPositionSetpoint);
    mSpindexerController.updateSmartDashboard();
    SmartDashboard.putNumber("Pickup Velocity Setpoint", mPickupVelocitySetpoint);
    mPickupController.updateSmartDashboard();
  }
}
