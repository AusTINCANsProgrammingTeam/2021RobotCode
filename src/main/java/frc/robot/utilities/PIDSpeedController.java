// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedController;
import frc.robot.MotorController;

/** Add your docs here. */
public class PIDSpeedController implements SpeedController {
    private MotorController mMotorController;
    private int mMaxRPM;
    private double velocity; //RPM
    public PIDSpeedController(MotorController motorController, int maxRPM) {
        mMotorController = motorController;
        mMaxRPM = maxRPM;
    }
    
    /**
    * Common interface for setting the speed of a speed controller.
    *
    * @param speed The speed to set. Value should be between -1.0 and 1.0.
    */
    public void set(double speed) {
        double velocity = speed * mMaxRPM;
        this.velocity = velocity;
        mMotorController.getPID().setReference(velocity, ControlType.kVelocity);
    }

    /**
    * Sets the voltage output of the SpeedController. Compensates for the current bus voltage to
    * ensure that the desired voltage is output even if the battery voltage is below 12V - highly
    * useful when the voltage outputs are "meaningful" (e.g. they come from a feedforward
    * calculation).
    *
    * <p>NOTE: This function *must* be called regularly in order for voltage compensation to work
    * properly - unlike the ordinary set function, it is not "set it and forget it."
    *
    * @param outputVolts The voltage to output.
    */
    public void setVoltage(double outputVolts) {
        mMotorController.getPID().setReference(outputVolts, ControlType.kVoltage);
    }
   
    /**
    * Common interface for getting the current set speed of a speed controller.
    *
    * @return The current set speed. Value is between -1.0 and 1.0.
    */
    public double get() {
        return velocity / mMaxRPM;
    }

    /**
    * Common interface for inverting direction of a speed controller.
    *
    * @param isInverted The state of inversion true is inverted.
    */
    public void setInverted(boolean isInverted) {
        mMotorController.getSparkMax().setInverted(isInverted);
    }

    /**
    * Common interface for returning if a speed controller is in the inverted state or not.
    *
    * @return isInverted The state of the inversion true is inverted.
    */
    public boolean getInverted() {
        return mMotorController.getSparkMax().getInverted();
    }
    
    
    /** Disable the speed controller. */
    public void disable() {
        mMotorController.getPID().setReference(0.0, ControlType.kVoltage);
    }

    /**
    * Stops motor movement. Motor can be moved again by calling set without having to re-enable the
    * motor.
    */
    public void stopMotor() {
        mMotorController.getPID().setReference(0.0, ControlType.kVoltage);
    }

    @Override
    public void pidWrite(double output) {
        set(output);
    }
}
