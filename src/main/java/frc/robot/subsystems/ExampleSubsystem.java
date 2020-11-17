/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.lang.Class;
import java.lang.reflect.InvocationTargetException;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import net.thefletcher.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//import net.thefletcher.revrobotics.enums.MotorType;

public class ExampleSubsystem extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  Object m_frontSparkMax;
  String sparkMaxLib;
  Class<?> motorTypeClass;

  public ExampleSubsystem(boolean isReal) {
    String sparkMaxMotorTypeLib;
    if (isReal) {
      sparkMaxLib = "com.revrobotics.CANSparkMax";
      sparkMaxMotorTypeLib = "com.revrobotics.CANSparkMaxLowLevel.MotorType";
    } else {

      sparkMaxLib = "net.thefletcher.revrobotics.CANSparkMax";
      sparkMaxMotorTypeLib = "net.thefletcher.revrobotics.enums.MotorType";
    }

    try {
      motorTypeClass = this.getClass().getClassLoader().loadClass(sparkMaxMotorTypeLib);

      m_frontSparkMax = this.getClass().getClassLoader().loadClass(sparkMaxLib)
          .getDeclaredConstructor(new Class[] { Integer.TYPE, motorTypeClass })
          .newInstance(new Object[] { 0, motorTypeClass.getEnumConstants()[1] });
    } catch (InstantiationException | IllegalAccessException | IllegalArgumentException | InvocationTargetException
        | NoSuchMethodException | SecurityException | ClassNotFoundException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    try {
      m_frontSparkMax.getClass().getMethod("set", new Class[] { Double.TYPE }).invoke(m_frontSparkMax,
          new Object[] { 0.5 });
    } catch (IllegalAccessException | IllegalArgumentException | InvocationTargetException | NoSuchMethodException
        | SecurityException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
  }
}
