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
        mSparkMax = new CANSparkMax(deviceID, MotorType.kBrushless);
        //Initializing encoder
        //We can only call .getEncoder once because of a bug in spark max api
        //which causes encoder initialization to occur on every call of .getEncoder() this breaks things
        mEncoder = mSparkMax.getEncoder();
        mSparkMax.restoreFactoryDefaults();
    }

    public MotorController(String name, int deviceID, int smartCurrentLimit) {
        this(name, deviceID);
        //Current limiting is required to prevent brown outs
        mSparkMax.setSmartCurrentLimit(smartCurrentLimit);
    }

    public MotorController(String name, int deviceID, int smartCurrentLimit, boolean enablePid) {
        this(name, deviceID, smartCurrentLimit, SmartDashboard.getNumber(name + " P Value", 1.0), 
            SmartDashboard.getNumber(name + " I Value", 0.0), SmartDashboard.getNumber(name + " D Value", 0.0)); //calls the constructor for correct arguments
    }

    public MotorController(String name, int deviceID, int smartCurrentLimit, double P, double I, double D) {
        this(name, deviceID, smartCurrentLimit); //calls the constructor for correct arguements
        mP = P;
        mI = I;
        mD = D;
        mPIDController = mSparkMax.getPIDController();
        setPID();
        mSparkMax.setOpenLoopRampRate(.1);
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
