// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

public class DriveTo extends CommandBase{
    private SwerveDrive swerve;
    private ProfiledPIDController xController ;
    private ProfiledPIDController yController ;
    private ProfiledPIDController zController ;

    private static double x_P;
    private static double x_I;
    private static double x_D;
    private static double y_P;
    private static double y_I;
    private static double y_D;
    private static double z_P;
    private static double z_I;
    private static double z_D;
    private static double maxSpeed;
    private static double maxRotationSpeed;

    static {
        Preferences.initDouble("DriveTo/X/P",0.8);
        x_P = Preferences.getDouble("DriveTo/X/P",0.8);
        Preferences.initDouble("DriveTo/X/I",0);
        x_I = Preferences.getDouble("DriveTo/X/I",0);
        Preferences.initDouble("DriveTo/X/D",0);
        x_D = Preferences.getDouble("DriveTo/X/D",0);

        Preferences.initDouble("DriveTo/Y/P",0.8);
        y_P = Preferences.getDouble("DriveTo/Y/P",0.8);
        Preferences.initDouble("DriveTo/Y/I",0);
        y_I = Preferences.getDouble("DriveTo/Y/I",0);
        Preferences.initDouble("DriveTo/Y/D",0);
        y_D = Preferences.getDouble("DriveTo/Y/D",0);

        Preferences.initDouble("DriveTo/Z/P",0.8);
        z_P = Preferences.getDouble("DriveTo/Z/P",0.8);
        Preferences.initDouble("DriveTo/Z/I",0);
        z_I = Preferences.getDouble("DriveTo/Z/I",0);
        Preferences.initDouble("DriveTo/Z/D",0);
        z_D = Preferences.getDouble("DriveTo/Z/D",0);

        Preferences.initDouble("DriveTo/Speed/X_Y",1);
        maxSpeed = Preferences.getDouble("DriveTo/Speed/X_Y",1);
        Preferences.initDouble("DriveTo/Speed/Z",10);
        maxRotationSpeed = Preferences.getDouble("DriveTo/Speed/Z",10);
    } ;

    public DriveTo(SwerveDrive swerve, double x, double y, double z, boolean isRelative){ 
        addRequirements(swerve);
        this.swerve = swerve;
        xController = new ProfiledPIDController(x_P, x_I, x_D, new TrapezoidProfile.Constraints(maxSpeed, maxSpeed/10.0));
        yController = new ProfiledPIDController(y_P, y_I, y_D, new TrapezoidProfile.Constraints(maxSpeed, maxSpeed/10.0));
        zController = new ProfiledPIDController(z_P, z_I, z_D, new TrapezoidProfile.Constraints(maxRotationSpeed, maxRotationSpeed/10.0));

        zController.enableContinuousInput(-180.0, 180.0);

        xController.setTolerance(0.1);
        yController.setTolerance(0.1);
        zController.setTolerance(1.0);

        if (isRelative) {
            Pose2d pose = swerve.getPose().transformBy( new Transform2d( new Translation2d(x,y), Rotation2d.fromDegrees(z) ) );
            xController.setGoal( pose.getX() );
            yController.setGoal( pose.getY() );
            zController.setGoal( pose.getRotation().getDegrees() );
        } else {
            xController.setGoal( x );
            yController.setGoal( y );
            zController.setGoal( z );
        }
    }

    public void initialize() {
        Pose2d pose = swerve.getPose();
        xController.reset( pose.getX() );
        yController.reset( pose.getY() );
        zController.reset( pose.getRotation().getDegrees() );
    }
    
    public void execute() {
        Pose2d pose = swerve.getPose();
        double xSpeed = xController.calculate( pose.getX() );
        double ySpeed = yController.calculate( pose.getY() );
        double zSpeed = zController.calculate( pose.getRotation().getDegrees() ); 
        
        SmartDashboard.putNumber("DriveTo/Error/x", xController.getPositionError());
        SmartDashboard.putNumber("DriveTo/Error/y", yController.getPositionError());
        SmartDashboard.putNumber("DriveTo/Error/z", zController.getPositionError());
        
        swerve.driveSpeed( xSpeed, ySpeed, zSpeed, true );
    }
    public boolean isFinished() {
      return xController.atGoal() && yController.atGoal() && zController.atGoal() ;
    }

    public void end(boolean interrupted) {
        swerve.driveSpeed(0, 0, 0, true);
    }
  
}