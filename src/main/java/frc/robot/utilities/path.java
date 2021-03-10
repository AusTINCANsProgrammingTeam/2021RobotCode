package frc.robot.utilities;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import java.io.*;
import java.nio.file.Path;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveBaseSubsystem;

public class path {
    private String mPathName;
    private RamseteCommand mPathCommand;
    public Trajectory mTrajectory = new Trajectory();

    private final DriveBaseSubsystem mDriveBaseSubsystem;

    public path(String pathName, DriveBaseSubsystem driveBaseSubsystem) {
        mPathName = pathName;
        mDriveBaseSubsystem = driveBaseSubsystem;
        try {
            Path path = Filesystem.getDeployDirectory().toPath().resolve(mPathName);
            mTrajectory = TrajectoryUtil.fromPathweaverJson(path);
        } catch (IOException ex) {
            DriverStation.reportError("Could not find file: " + mPathName, ex.getStackTrace());
        }
        mPathCommand = new RamseteCommand(mTrajectory, mDriveBaseSubsystem,
                new RamseteController() /* b = 2.0, zeta = 0.7 */,
                mDriveBaseSubsystem.getDifferentialDriveKinematics(), mDriveBaseSubsystem, mDriveBaseSubsystem);

    }
}
