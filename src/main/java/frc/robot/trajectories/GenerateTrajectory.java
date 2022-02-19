// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.trajectories;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;

/** Add your docs here. */
public class GenerateTrajectory { 

    public static final double maxSpeed = 1; 
    public static final double maxAcceleration = .1;
    private GenerateTrajectory(){} 

    private static TrajectoryConfig config = new TrajectoryConfig(maxSpeed, maxAcceleration)
                                                    .setKinematics(Constants.SwerveBase.kinematics);

    public static Trajectory backupOneMeter(Pose2d currentPosition)
    { 
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory( 
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(0, Units.feetToMeters(1)), new Translation2d(0, Units.feetToMeters(2))),
            // End 3 feet straight ahead of where we started, facing forward
            new Pose2d(Units.feetToMeters(0), Units.feetToMeters(3), new Rotation2d(0)),
            config);
        return trajectory.relativeTo(currentPosition); 
    }
}
