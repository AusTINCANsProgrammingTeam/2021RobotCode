/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMax;
import com.revrobotics.CANSparkMaxLowLevel.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class MotorController {
    private String mName;
    private CANSparkMax mSparkMax;
    private CANEncoder mEncoder;
    private CANPIDController mPIDController = null;
    private double mP;
    private double mI;
    private double mD;

    public MotorController(String name, int deviceID) {
        mName = name;
        mSparkMax = new CANSparkMaxWrap(deviceID, MotorType.kBrushless);
        //Initializing encoder
        //We can only call .getEncoder once because of a bug in spark max api
        //which causes encoder initialization to occur on every call of .getEncoder() this breaks things
        mEncoder = mSparkMax.getEncoder();
        mSparkMax.restoreFactoryDefaults();
    }

    //Todo: we must add constants for PID values once we have found the right values
    public MotorController(String name, int deviceID, int smartCurrentLimit, boolean... enablePid) {
        this(name, deviceID); //calls the constructor for correct arguements
        //Current limiting is required to prevent brown outs
        mSparkMax.setSmartCurrentLimit(smartCurrentLimit);
        //If enablePid has any number of booleans greater than 0 we are enabling pid
        if (enablePid.length > 0)
        {
            mP = SmartDashboard.getNumber(mName + " P Value", 1.0);
            mI = SmartDashboard.getNumber(mName + " I Value", 0.0);
            mD = SmartDashboard.getNumber(mName + " D Value", 0.0);
            mPIDController = mSparkMax.getPIDController();
            setPID();
        }
    }

    public CANSparkMax getSparkMax() {
        return mSparkMax;
    }

    public CANEncoder getEncoder() {
        return mEncoder;
    }

    public CANPIDController getPID() {
        return mPIDController;
    }

    public void setPID() {
        mPIDController.setP(mP);
        mPIDController.setI(mI);
        mPIDController.setD(mD);
        SmartDashboard.putNumber(mName+" P Value", mP);
        SmartDashboard.putNumber(mName+" I Value", mI);
        SmartDashboard.putNumber(mName+" D Value", mD);
    }

    public void updateSmartDashboard() {
        //The simulation crashes whenever .getEncoder() is called
        if(Robot.isReal()) {
            SmartDashboard.putNumber(mName + " Encoder Position", mEncoder.getPosition());
            SmartDashboard.putNumber(mName + " Encoder Velocity", mEncoder.getVelocity());
        }
        if(mPIDController != null) {
            if (SmartDashboard.getNumber(mName + " P Value", mP) != mP) {
                mP = SmartDashboard.getNumber(mName + " P Value", mP);
                mPIDController.setP(mP);
            }
            if (SmartDashboard.getNumber(mName + " I Value", mI) != mI) {
                mI = SmartDashboard.getNumber(mName + " I Value", mI);
                mPIDController.setI(mI);
            }
            if (SmartDashboard.getNumber(mName + " D Value", mD) != mD) {
                mD = SmartDashboard.getNumber(mName + " D Value", mD);
                mPIDController.setD(mD);
            }
        }
    }
    
}
