// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.trajectories;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class GenerateTrajectory { 

    public static final double maxSpeed = 1; 
    public static final double maxAcceleration = .1;
    private GenerateTrajectory(){} 

    private static TrajectoryConfig baseConfig(){
        return new TrajectoryConfig(maxSpeed, maxAcceleration)
                        .setKinematics(Constants.SwerveBase.KINEMATICS);
    }

    public static Trajectory backupOneMeter(Pose2d currentPosition) { 
        TrajectoryConfig config = baseConfig().setReversed(true);
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory( 
            new Pose2d( 0, 0, new Rotation2d() ),
            List.of(),
            new Pose2d( -1, 0, new Rotation2d() ),
            config);
        return trajectory.transformBy( new Transform2d( trajectory.getInitialPose(), currentPosition )); 
    } 

    public static Trajectory driveTo(Pose2d currentPosition, Pose2d targetPosition){
        TrajectoryConfig config = baseConfig();
        Rotation2d facing = calculateAngle(currentPosition, targetPosition);
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory( 
            new Pose2d(currentPosition.getX(), currentPosition.getY(), facing),
            List.of(),
            new Pose2d(targetPosition.getX(), targetPosition.getY(), facing),
            config);        
        return trajectory;
    }

    private static Rotation2d calculateAngle(Pose2d p1, Pose2d p2)
    {
        double degrees = Math.toDegrees(Math.atan2(p2.getX() - p1.getX(), p2.getY() - p1.getY()));
        degrees = degrees + Math.ceil( -degrees / 360 ) * 360;
    
        return Rotation2d.fromDegrees(degrees);
    }
    // public static Trajectory twoBallAutoLeftPartOne(Pose2d currentPosition){ 
    //     Trajectory trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)), 
    //     List.of(new Translation2d(0, Units.feetToMeters(1)), new Translation2d(0, Units.feetToMeters(2))), new Pose2d(), config)
    // }
}