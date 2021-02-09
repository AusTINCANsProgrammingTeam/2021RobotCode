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
  private final MotorController mSpindexerController;
  private final MotorController mPickupController;

  private double mSpindexerPositionSetpoint = 0.0;
  private double mPickupVelocitySetpoint = 0.0;

  public HopperSubsystem() {
    mSpindexerController = new MotorController("Spindexer", Constants.kHopperSpindexerPort, 
      Constants.kHopperCurrentLimit, true);

    mPickupController = new MotorController("Hopper Pickup", Constants.kHopperPickupMotorPort, 
      Constants.kHopperCurrentLimit, true);

  }

  public void setSpindexerVelocity(double velocity) {
    mSpindexerPositionSetpoint = velocity;
    mSpindexerController.getPID().setReference(velocity, ControlType.kVelocity);
  }

  public void setPickupVelocity(double velocity) {
    mPickupVelocitySetpoint = velocity;
    mPickupController.getPID().setReference(velocity, ControlType.kVelocity);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Spindexer Position Setpoint", mSpindexerPositionSetpoint);
    mSpindexerController.updateSmartDashboard();
    SmartDashboard.putNumber("Hopper Pickup Velocity Setpoint", mPickupVelocitySetpoint);
    mPickupController.updateSmartDashboard();
  }
}
