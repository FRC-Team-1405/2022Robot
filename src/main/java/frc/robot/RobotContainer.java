// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
  private final SwerveDrive driveBase = new SwerveDrive(); 

  private XboxController driver = new XboxController(1);

  public RobotContainer() {
    configureButtonBindings();

    driveBase.setDefaultCommand(new SwerveDriveCommand(this::getXSpeed, 
                                                      this::getYSpeed, 
                                                      this::getRotationSpeed, driveBase));
  }

  public double getXSpeed(){ 
    return driver.getLeftY();
  } 

  public double getYSpeed(){ 
    return -driver.getLeftX();
  } 

  public double getRotationSpeed(){ 
    return driver.getRightX(); 
  }


  private void configureButtonBindings() {}

  public Command getAutonomousCommand() {
    return null;
  }
}
