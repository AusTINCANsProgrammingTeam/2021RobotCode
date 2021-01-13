/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.CANSparkMaxWrap;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.*;

public class ExampleSubsystem extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */

  private final CANSparkMax[] test = new CANSparkMax[4];
  private final DifferentialDrive diffDrive;

  public ExampleSubsystem() {
    for (int i = 0; i < test.length; i++) {
      test[i] = new CANSparkMaxWrap(i, MotorType.kBrushless);
    }
    test[1].follow(test[0]);
    test[3].follow(test[1]);
    diffDrive = new DifferentialDrive(test[0], test[1]);
  }

  public void arcadeDrive(Joystick js) {
    diffDrive.arcadeDrive(js.getRawAxis(0), js.getRawAxis(3));
  }

  @Override
  public void periodic() {
  }
}
