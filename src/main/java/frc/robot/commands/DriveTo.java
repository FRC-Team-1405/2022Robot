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

    static {
        Preferences.initDouble("DriveTo/X/P",1.0);
        x_P = Preferences.getDouble("DriveTo/X/P",1);
        Preferences.initDouble("DriveTo/X/I",0.01);
        x_I = Preferences.getDouble("DriveTo/X/I",0.01);
        Preferences.initDouble("DriveTo/X/D",0);
        x_D = Preferences.getDouble("DriveTo/X/D",0);

        Preferences.initDouble("DriveTo/Y/P",1.0);
        y_P = Preferences.getDouble("DriveTo/Y/P",1);
        Preferences.initDouble("DriveTo/Y/I",0.01);
        y_I = Preferences.getDouble("DriveTo/Y/I",0.01);
        Preferences.initDouble("DriveTo/Y/D",0);
        y_D = Preferences.getDouble("DriveTo/Y/D",0);

        Preferences.initDouble("DriveTo/Z/P",1.0);
        z_P = Preferences.getDouble("DriveTo/Z/P",1);
        Preferences.initDouble("DriveTo/Z/I",0.01);
        z_I = Preferences.getDouble("DriveTo/Z/I",0.01);
        Preferences.initDouble("DriveTo/Z/D",0);
        z_D = Preferences.getDouble("DriveTo/Z/D",0);
    } ;

    public DriveTo(SwerveDrive swerve, double x, double y, double z, boolean isRelative){ 
        this.swerve = swerve;
        xController = new ProfiledPIDController(x_P, x_I, x_D, new TrapezoidProfile.Constraints(swerve.maxVelocity, swerve.maxVelocity/3.0));
        yController = new ProfiledPIDController(y_P, y_I, y_D, new TrapezoidProfile.Constraints(swerve.maxVelocity, swerve.maxVelocity/3.0));
        zController = new ProfiledPIDController(z_P, z_I, z_D, new TrapezoidProfile.Constraints(swerve.maxAngularSpeed, swerve.maxAngularSpeed/3.0));

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
        swerve.drive( xController.calculate( pose.getX() ), 
                      yController.calculate( pose.getY() ),
                      zController.calculate( pose.getRotation().getDegrees() ) );
    }
    public boolean isFinished() {
      return xController.atGoal() && yController.atGoal() && zController.atGoal() ;
    }

    public void end(boolean interrupted) {
        swerve.drive(0, 0, 0);
    }
  
}