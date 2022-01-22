// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
  private final SwerveDrive driveBase = new SwerveDrive(); 
  private final Shooter shooter = new Shooter(); 
  private final Intake intake = new Intake();
  
  private XboxController driver = new XboxController(Constants.Controller.DRIVER);
  private XboxController tester = new XboxController(Constants.Controller.TESTER);


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


  private void configureButtonBindings() {
    configureTester();
  }

  private void configureTester() {
    new JoystickButton(tester, XboxController.Button.kY.value)
          .whenHeld( new InstantCommand( shooter::flywheelHighSpeed, shooter ))
          .whenReleased( new InstantCommand( shooter::flywheelStop, shooter)) ;
    new JoystickButton(tester, XboxController.Button.kA.value)
          .whenHeld(new InstantCommand( shooter::flywheelLowSpeed, shooter  ))
          .whenReleased(new InstantCommand( shooter::flywheelStop, shooter)); 
    new JoystickButton(tester, XboxController.Button.kRightBumper.value)
          .toggleWhenPressed( new StartEndCommand(intake::dropIntake, intake::liftIntake, intake ));

  }

  public Command getAutonomousCommand() {
    return null;
  }
}
