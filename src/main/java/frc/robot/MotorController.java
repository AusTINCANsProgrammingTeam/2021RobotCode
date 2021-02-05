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

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class MotorController {
    private String mName;
    private CANSparkMax mSparkMax;
    private CANEncoder mEncoder;
    private CANPIDController mPIDController;
    private int mP;
    private int mI;
    private int mD;

    public MotorController(String name, int deviceID) {
        mName = name;
        mSparkMax = new CANSparkMaxWrap(deviceID, MotorType.kBrushless);
        //Initializing encoder
        //We can only call .getEncoder once because of a bug in spark max api
        //which causes encoder initialization to occur on every call of .getEncoder() this breaks things
        mEncoder = mSparkMax.getEncoder();
        mSparkMax.restoreFactoryDefaults();
    }

    public MotorController(String name, int deviceID, int smartCurrentLimit) {
        this(name, deviceID); //calls the constructor for correct arguements
        //Current limiting is required to prevent brown outs
        mSparkMax.setSmartCurrentLimit(smartCurrentLimit);
    }
    
    public MotorController(String name, int deviceID, int smartCurrentLimit, int p, int i, int d) {
        this(name, deviceID, smartCurrentLimit); //calls the constructor for correct arguements
        mP = p;
        mI = i;
        mD = d;
        mPIDController = mSparkMax.getPIDController();
    }


    public CANSparkMax getSparkMax() {
        return mSparkMax;
    }

    public CANEncoder getEncoder() {
        return mEncoder;
    }

    public void updateSmartDashboard() {
        //The simulation crashes whenever .getEncoder() is called
        if(Robot.isReal()) {
            SmartDashboard.putNumber(mName + " Encoder Position", mEncoder.getPosition());
            SmartDashboard.putNumber(mName + " Encoder Velocity", mEncoder.getVelocity());
        }
    }
    
}
