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
    public static final int kShooterMotorPort = 0;

    public static final int kHoodDoubleSolenoidForwardChannel = 1;
    public static final int kHoodDoubleSolenoidReverseChannel = 2;

    // Must be 1-10
    public static final int kAButton = 1;
    public static final int kBButton = 2;
    public static final int kXButton = 3;
    public static final int kYButton = 4;

    public  static final int kShooterMotorCurrentLimit = 40;

    public static final double kShooterVelocityRange = 0.025;
    public static final double kHoodExtendRequiredDistance = 20.0;
    // The zones are in inches. These variables describe the rightmost bound
    public static final double kGreenShootingZone = 90.0;
    public static final double kYellowShootingZone = 150.0;
    public static final double kBlueShootingZone = 210.0;
    public static final double kRedShootingZone = 270.0;
    
    public static final int kJoystickPort = 0;
}
