/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.MotorController;

public class HopperSubsystem extends SubsystemBase {
  /**
   * Creates a new HopperSubsystem.
   */
  private final MotorController mHopperV1Controller;
  private final MotorController mHopperV2Controller;
  private final MotorController mPickupController;

  private double mVSpeed = Constants.kHopperVSpinPercentage;
  private double mPickupSpeed = Constants.kHopperVSpinPercentage;

  public HopperSubsystem() {
    mHopperV1Controller = new MotorController("1st Hopper Powered V", Constants.kHopperV1Port, 
      Constants.kHopperCurrentLimit);
    mHopperV2Controller = new MotorController("2nd Hopper Powered V", Constants.kHopperV2Port, 
    Constants.kHopperCurrentLimit);
    mPickupController = new MotorController("Hopper Pickup", Constants.kHopperPickupMotorPort,
      Constants.kHopperCurrentLimit);
    mPickupController.getSparkMax().setInverted(true);
  }

  public void setPickup() {
    mPickupController.getSparkMax().set(mPickupSpeed);
  }

  public void setPoweredV() {
    mHopperV1Controller.getSparkMax().set(mVSpeed);
    mHopperV2Controller.getSparkMax().set(mVSpeed);
  }

  public void stopAllMotors() {
    mPickupController.getSparkMax().set(0.0);
    mHopperV1Controller.getSparkMax().set(0.0);
    mHopperV2Controller.getSparkMax().set(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    mHopperV1Controller.updateSmartDashboard();
    mHopperV2Controller.updateSmartDashboard();
    mPickupController.updateSmartDashboard();
    mVSpeed = SmartDashboard.getNumber("Hopper Powered V Motor Speed", mVSpeed);
    mPickupSpeed = SmartDashboard.getNumber("Hopper Pickup Motor Speed", mPickupSpeed);
  }
}
