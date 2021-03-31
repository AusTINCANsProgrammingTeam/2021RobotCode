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
    public static final int kShooterMotorPort = 10;

    public static final int kHoodDoubleSolenoidForwardChannel = 1;
    public static final int kHoodDoubleSolenoidReverseChannel = 5;

    // Must be 1-10
    public static final int kAButton = 1;
    public static final int kBButton = 2;
    public static final int kXButton = 3;
    public static final int kYButton = 4;
    public static final int kRightBumperButton = 5;
    public static final int kLeftBumperButton = 6;
    public static final int kRightTriggerButton = 7;
    public static final int kLeftTriggerButton = 8;

    public  static final int kShooterMotorCurrentLimit = 40;

    public static final double kShooterVelocityPlusMinusPercent = 0.025;
    public static final double kHoodExtendRequiredDistance = 20.0; //inches
    // The zones are in inches. These variables describe the rightmost bound
    public static final double kGreenShootingZone = 90.0;
    public static final double kYellowShootingZone = 150.0;
    public static final double kBlueShootingZone = 210.0;
    public static final double kRedShootingZone = 270.0;
    
    public static final int kJoystickPort = 0;
    
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
    
    // Controller in the lab is Logitech F310 USB Gamepad
    public static final int kLeftJoystickAxisX = 0;
    public static final int kLeftJoystickAxisY = 1;
    public static final int kRightJoystickAxisX = 2;
    public static final int kRightJoystickAxisY = 3;

    public static final int kHopperV1Port = 4; 
    public static final int kHopperV2Port = 11;
    public static final int kHopperPickupMotorPort = 9;

    public static final double kHopperVSpinPercentage = 0.3;
    public static final double kHopperPickupPercentage = 0.2;

    public static final double kHopperSpinPercentage = 1.0; //Todo: set limit

    public static final int kHopperCurrentLimit = 40; //Todo: may change
    public static final int kIntakeDoubleSolenoidForwardChannel = 0;
    public static final int kIntakeDoubleSolenoidReverseChannel = 2;

    public static final int kIntakeDeviceID = 5;
    public static final double kIntakeMotorRunPercent = 1.0;
} 
