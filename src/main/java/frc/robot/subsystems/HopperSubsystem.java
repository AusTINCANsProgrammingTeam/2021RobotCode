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
  private final MotorController mElevatorController;

  private final CANPIDController mSpindexerPID;
  private final CANEncoder mSpindexerEncoder;
  private double mSpindexerP = 0.0;
  private double mSpindexerI = 0.0;
  private double mSpindexerD = 0.0;

  private final CANPIDController mElevatorPID;
  private double mElevatorP = 0.0;
  private double mElevatorI = 0.0;
  private double mElevatorD = 0.0;

  private double mSpindexerPositionSetpoint = 0.0;
  private double mElevatorVelocitySetpoint = 0.0;

  public HopperSubsystem() { //possibility of needing PID (?)
    mSpindexerController = new MotorController("", Constants.kHopperSpindexerPort); //Todo: set name
    mSpindexerPID = mSpindexerController.getSparkMax().getPIDController();
    mSpindexerPID.setP(mSpindexerP);
    mSpindexerPID.setI(mSpindexerI);
    mSpindexerPID.setD(mSpindexerD);
    mSpindexerEncoder = mSpindexerController.getEncoder();
    //Convert position value from NEO rotations to spindexer rotations
    mSpindexerEncoder.setPositionConversionFactor(1.0/Constants.kSpindexerGearRatio);

    SmartDashboard.putNumber("Spindexer P Value", mSpindexerP);
    SmartDashboard.putNumber("Spindexer I Value", mSpindexerI);
    SmartDashboard.putNumber("Spindexer D Value", mSpindexerD);
    mElevatorController = new MotorController("", Constants.kHopperElevatorMotorPort); //Todo: set name
    mElevatorPID = mElevatorController.getSparkMax().getPIDController();
    mElevatorPID.setP(mElevatorP);
    mElevatorPID.setI(mElevatorI);
    mElevatorPID.setD(mElevatorD);
    SmartDashboard.putNumber("Elevator P Value", mElevatorP);
    SmartDashboard.putNumber("Elevator I Value", mElevatorI);
    SmartDashboard.putNumber("Elevator D Value", mElevatorD);

  }

  public void setSpindexerPosition(double position) {
    mSpindexerPositionSetpoint = position;
    mSpindexerPID.setReference(position, ControlType.kPosition);
  }

  public void incrementSpindexPosition(double increment) {
    setSpindexerPosition(mSpindexerPositionSetpoint + increment);
  }

  public void setElevatorVelocity(double velocity) {
    mElevatorVelocitySetpoint = velocity;
    mElevatorPID.setReference(velocity, ControlType.kVelocity);
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

    if (SmartDashboard.getNumber("Elevator P Value", mElevatorP) != mElevatorP) {
      mElevatorP = SmartDashboard.getNumber("Elevator P Value", mElevatorP);
      mElevatorPID.setP(mElevatorP);
    }
    if (SmartDashboard.getNumber("Elevator I Value", mElevatorI) != mElevatorI) {
      mElevatorI = SmartDashboard.getNumber("Elevator I Value", mElevatorI);
      mElevatorPID.setP(mElevatorI);
    }
    if (SmartDashboard.getNumber("Elevator D Value", mElevatorD) != mElevatorD) {
      mElevatorD = SmartDashboard.getNumber("Elevator D Value", mElevatorD);
      mElevatorPID.setP(mElevatorD);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updatePIDFromSmartDashboard();
    SmartDashboard.putNumber("Spindexer Position Setpoint", mSpindexerPositionSetpoint);
    mSpindexerController.updateSmartDashboard();
    SmartDashboard.putNumber("Elevator Velocity Setpoint", mElevatorVelocitySetpoint);
    mElevatorController.updateSmartDashboard();
  }
}
