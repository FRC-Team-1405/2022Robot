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
import frc.robot.subsystems.SwerveSubsystem;

public class GenerateTrajectory { 

    public static final double maxSpeed = 1; 
    public static final double maxAcceleration = .1;
    private GenerateTrajectory(){} 

    private static TrajectoryConfig baseConfig(SwerveSubsystem swerve){
        return new TrajectoryConfig(swerve.getMaxSpeed(), swerve.getMaxAcceleration())
                        .setKinematics(swerve.getKinematics());
    }

    public static Trajectory backupOneMeter(Pose2d currentPosition, SwerveSubsystem swerve) { 
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory( 
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(0, Units.feetToMeters(-1)), new Translation2d(0, Units.feetToMeters(-2))),
            // End 3 feet straight ahead of where we started, facing forward
            new Pose2d(Units.feetToMeters(0), Units.feetToMeters(-3), new Rotation2d(0)),
            baseConfig(swerve));
        return trajectory.relativeTo(currentPosition); 
    } 

    public static Trajectory driveTo(Pose2d currentPosition, Pose2d targetPosition, SwerveSubsystem swerve) {
        TrajectoryConfig config = baseConfig(swerve);
        Rotation2d facing = calculateAngle(currentPosition, targetPosition);
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory( 
            new Pose2d(currentPosition.getX(), currentPosition.getY(), facing),
            List.of(),
            new Pose2d(targetPosition.getX(), targetPosition.getY(), facing),
            config);        
        return trajectory;
    }

    public static Rotation2d calculateAngle(Pose2d p1, Pose2d p2)
    {
        double degrees = Math.toDegrees(Math.atan2( p2.getY() - p1.getY(), p2.getX() - p1.getX()));    
        return Rotation2d.fromDegrees(degrees);
    }

    public static Pose2d pose2dToOrigin(double x, double y){
        double degrees = Math.toDegrees(Math.atan2(x, y)) + 90.0;    
        return new Pose2d(x, y, Rotation2d.fromDegrees(degrees));
    }

    public static Pose2d pose2dToOrigin(double x, double y, boolean flip){
        if (flip) {
            // double angle = -pose2dToOrigin(y, x).getRotation().getDegrees() + 90; 
            double angle = pose2dToOrigin(x, y).getRotation().getDegrees() + 180; 
            return new Pose2d(x, y, Rotation2d.fromDegrees(angle));
        }
        return pose2dToOrigin(y, x) ; 
    }
}
