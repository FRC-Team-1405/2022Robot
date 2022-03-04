// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveDrive;

public class DriveToTest extends SequentialCommandGroup{

    static {
        SmartDashboard.putNumber("DriveToTest/X", 0.0);
        SmartDashboard.putNumber("DriveToTest/Y", 0.0);
        SmartDashboard.putNumber("DriveToTest/Z", 0.0);
    }
    public DriveToTest(SwerveDrive swerve){ 
        addCommands( new DriveTo(swerve, 
                                 SmartDashboard.getNumber("DriveToTest/X", 0.0),
                                 SmartDashboard.getNumber("DriveToTest/Y", 0.0),
                                 SmartDashboard.getNumber("DriveToTest/Z", 0.0),
                                 true));
    }  
}