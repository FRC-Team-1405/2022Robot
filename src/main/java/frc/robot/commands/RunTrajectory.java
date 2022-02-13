// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;

/** Add your docs here. */
public class RunTrajectory extends SequentialCommandGroup {

    ProfiledPIDController thetaController =
    new ProfiledPIDController(
        0,
        0, 
        0, 
        new TrapezoidProfile.Constraints(Constants.SwerveBase.maxAngularSpeed, 
                                            Constants.SwerveBase.maxAngularAccelerartion));

public RunTrajectory(Trajectory trajectory, SwerveDrive swerveDrive){ 
    thetaController.enableContinuousInput(-Math.PI, Math.PI); 

    SwerveControllerCommand swerveControllerCommand =
    new SwerveControllerCommand(
        trajectory,
        swerveDrive::getPose, // Functional interface to feed supplier
        swerveDrive.getKinematics(),

        // Position controllers
        new PIDController(0, 0, 0),  
        new PIDController(0, 0, 0), 
                            
        thetaController,
        swerveDrive::setModuleStates,
        swerveDrive); 
}

}
