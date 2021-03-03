package frc.robot.utilities;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import java.io.*;
import java.nio.file.Path;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

public class path {
    private String mPathName;
    private RamseteCommand mPathCommand;
    public Trajectory mTrajectory = new Trajectory();
    

    public path(String pathName) {
        mPathName = pathName;
        try {
            Path path = Filesystem.getDeployDirectory().toPath().resolve(mPathName);
            mTrajectory = TrajectoryUtil.fromPathweaverJson(path);
        } 
        catch(IOException ex) {
            DriverStation.reportError("Could not find file: " + mPathName, ex.getStackTrace());
        }
        mPathCommand = new RamseteCommand(mTrajectory, pose, follower, 
            kinematics, outputMetersPerSecond, requirements);

    }
}
