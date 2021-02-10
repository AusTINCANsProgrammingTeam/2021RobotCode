/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    
    public static final int kDriveRightFront = 13;
    public static final int kDriveRightMiddle = 14;
    public static final int kDriveRightRear = 15;
    public static final int kDriveLeftFront = 1;
    public static final int kDriveLeftMiddle = 2;
    public static final int kDriveLeftRear = 16;

    public static final int kDriveLeftFrontIndex = 0;
    public static final int kDriveLeftMiddleIndex = 1;
    public static final int kDriveLeftRearIndex = 2;
    public static final int kDriveRightFrontIndex = 3;
    public static final int kDriveRightMiddleIndex = 4;
    public static final int kDriveRightRearIndex = 5;


    public static final int kDriveBaseCurrentLimit = 40;
    
    public static final int kLeftJoystickAxisX = 0;
    public static final int kLeftJoystickAxisY = 1;
    public static final int kRightJoystickAxisX = 4;
    public static final int kRightJoystickAxisY = 5;
    
    public static final int kJoystickPort = 0;

    public static final int kButtonA = 1;
    public static final int kButtonB = 2;
    public static final int kButtonX = 3;
    public static final int kButtonY = 4;

    public static final int kIntakeDoubleSolenoidForwardChannel = 1;
    public static final int kIntakeDoubleSolenoidReverseChannel = 2;

    public static final int kIntakeDeviceID = 1;
    public static final double kIntakeMotorMaxSpeed = 1.0;
} 
