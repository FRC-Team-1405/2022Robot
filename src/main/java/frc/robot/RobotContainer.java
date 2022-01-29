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
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  private final SwerveDrive driveBase = new SwerveDrive(); 
  private final Shooter shooter = new Shooter(); 
  private final Intake intake = new Intake();
  
  private XboxController driver = new XboxController(Constants.Controller.DRIVER);
  private XboxController operator = new XboxController(Constants.Controller.OPERATOR);


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
    return -driver.getRightX(); 
  }


  private void configureButtonBindings() {
    configureDriverButtons();
    configureOperatorButtons();
  }

  private void configureOperatorButtons(){
    var stopCommand = new InstantCommand( shooter::flywheelStop);
    var idleCommand = new InstantCommand( shooter::flywheelIdleSpeed);
    var toggleIdleCommand = new ConditionalCommand( idleCommand, stopCommand, shooter::isStopped);

    new JoystickButton(driver, XboxController.Button.kB.value).whenPressed( toggleIdleCommand );

    var upTrigger = new Trigger( () -> {
      return operator.getPOV() == 0 || operator.getPOV() == 45 || operator.getPOV() == 315;
    });
    var downTrigger = new Trigger( () -> {
      return operator.getPOV() == 180 || operator.getPOV() == 135 || operator.getPOV() == 225;
    });

    new JoystickButton(operator, XboxController.Button.kY.value)
        .and( upTrigger )
        .whenActive( shooter::increaseHighIndex );
    new JoystickButton(operator, XboxController.Button.kY.value)
        .and( downTrigger )
        .whenActive( shooter::decreaseHighIndex );
    new JoystickButton(operator, XboxController.Button.kA.value)
        .and( upTrigger )
        .whenActive( shooter::increaseLowIndex );
    new JoystickButton(operator, XboxController.Button.kA.value)
        .and( downTrigger )
        .whenActive( shooter::decreaseLowIndex );
  }   

  private void configureDriverButtons() {
    new JoystickButton(driver, XboxController.Button.kLeftStick.value)
        .whenPressed( new InstantCommand( () -> { driveBase.enableFieldOriented(true); }))
        .whenReleased( new InstantCommand( () -> { driveBase.enableFieldOriented(false); }));

  new JoystickButton(driver, XboxController.Button.kY.value)
        .whenHeld( new InstantCommand(shooter::flywheelHighSpeed, shooter ))
        .whenReleased( new InstantCommand(shooter::flywheelStop, shooter)) ;

  new JoystickButton(driver, XboxController.Button.kA.value)
        .whenHeld(new InstantCommand(shooter::flywheelLowSpeed, shooter  ))
        .whenReleased(new InstantCommand(shooter::flywheelStop, shooter)); 

  new JoystickButton(driver, XboxController.Button.kRightBumper.value)
        .toggleWhenPressed( new StartEndCommand(intake::intake, intake::intakeStop, intake ));
}

  public Command getAutonomousCommand() {
    return null;
  }
}