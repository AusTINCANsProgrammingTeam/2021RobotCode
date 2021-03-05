/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.MotorController;

public class IntakeSubsystem extends SubsystemBase {
  /**
   * Creates a new IntakeSubsystem.
   */
  //1 DS - extends/retracts intake, 1 neo - intake ball
  private final DoubleSolenoid mIntakeDoubleSolenoid;
  private final MotorController mIntakeController;

  public IntakeSubsystem() {
    mIntakeDoubleSolenoid = new DoubleSolenoid(Constants.kIntakeDoubleSolenoidForwardChannel, Constants.kIntakeDoubleSolenoidReverseChannel);
    mIntakeController = new MotorController("Intake", Constants.kIntakeDeviceID);
    mIntakeController.getSparkMax().setInverted(true);;
  }

  public void setIntakeExtended(boolean setExtended) {
    if(setExtended)
      mIntakeDoubleSolenoid.set(Value.kReverse);
    else
      mIntakeDoubleSolenoid.set(Value.kForward);
  }

  public void setIntakeExtended() {
    setIntakeExtended(true);
  }

  public void setIntakeRetracted() {
    setIntakeExtended(false);
  }
 
  public void setIntakeSpeed(double speed) {
    SmartDashboard.putNumber("Intake Speed Command", speed); 
    mIntakeController.getSparkMax().set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Intake Double Solenoid is extended", mIntakeDoubleSolenoid.get().equals(Value.kReverse));
    mIntakeController.updateSmartDashboard();
  }
}
