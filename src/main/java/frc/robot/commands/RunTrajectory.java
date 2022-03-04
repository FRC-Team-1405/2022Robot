// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.SwerveSubsystem;

public class RunTrajectory extends SequentialCommandGroup{    

    private static double x_P = 0.8 ;
    private static double x_I = 0.001 ;
    private static double x_D = 0.0 ;
    private static double y_P = 0.0 ;
    private static double y_I = 0.8 ;
    private static double y_D = 0.001 ;
    private static double z_P = 0.8 ;
    private static double z_I = 0.001 ;
    private static double z_D = 0.0 ;
    static{
        loadConfigs();
    }
    private PIDController xController;
    private PIDController yController;
    private ProfiledPIDController zController; 

    public RunTrajectory(Trajectory trajectory, SwerveSubsystem swerve){
        addRequirements(swerve);

        configPIDs(swerve);
        
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
            trajectory,
            swerve::getPose,
            swerve.getKinematics(),
            xController,
            yController,
            zController,
            swerve::setModuleStates,
            swerve);
    
        addCommands(swerveControllerCommand);        
    }

    public RunTrajectory(Trajectory trajectory, Supplier<Rotation2d> desiredRotation, SwerveSubsystem swerve){ 
        addRequirements(swerve);
        configPIDs(swerve);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
            trajectory,
            swerve::getPose,
            swerve.getKinematics(),
            xController,
            yController,
            zController,
            desiredRotation,
            swerve::setModuleStates,
            swerve);
    
        addCommands(swerveControllerCommand,
                    new InstantCommand( () -> { swerve.drive(0, 0, 0); } ));        
    }  

    private void configPIDs(SwerveSubsystem swerve){
        xController = new PIDController(x_P, x_I, x_D);
        xController.setTolerance(0.1);

        yController = new PIDController(y_P, y_I, y_D);
        yController.setTolerance(0.1);

        zController =  new ProfiledPIDController(z_P, z_I, z_D, new TrapezoidProfile.Constraints(swerve.getMaxAngularSpeed(),swerve.getMaxAngularAcceleration()));
        zController.enableContinuousInput(-Math.PI, Math.PI); 
        zController.setTolerance(Math.PI/100.0);
    }

    private static void loadConfigs(){
        Preferences.initDouble("RuntTrajectory/X/P", x_P);
        Preferences.initDouble("RuntTrajectory/X/I", x_I);
        Preferences.initDouble("RuntTrajectory/X/D", x_D);
        Preferences.initDouble("RuntTrajectory/Y/P", y_P);
        Preferences.initDouble("RuntTrajectory/Y/I", y_I);
        Preferences.initDouble("RuntTrajectory/Y/D", y_D);
        Preferences.initDouble("RuntTrajectory/Z/P", z_P);
        Preferences.initDouble("RuntTrajectory/Z/I", z_I);
        Preferences.initDouble("RuntTrajectory/Z/D", z_D);
        x_P = Preferences.getDouble("RuntTrajectory/X/P", x_P);
        x_I = Preferences.getDouble("RuntTrajectory/X/I", x_I);
        x_D = Preferences.getDouble("RuntTrajectory/X/D", x_D);
        y_P = Preferences.getDouble("RuntTrajectory/Y/P", y_P);
        y_I = Preferences.getDouble("RuntTrajectory/Y/I", y_I);
        y_D = Preferences.getDouble("RuntTrajectory/Y/D", y_D);
        z_P = Preferences.getDouble("RuntTrajectory/Z/P", z_P);
        z_I = Preferences.getDouble("RuntTrajectory/Z/I", z_I);
        z_D = Preferences.getDouble("RuntTrajectory/Z/D", z_D);
    }
}