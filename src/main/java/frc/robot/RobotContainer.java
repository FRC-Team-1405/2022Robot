// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.BatteryLED;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.sensor.LEDStrip;
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

  private LEDStrip ledStrip = new LEDStrip(Constants.PWM_Port.LEDS, Constants.PWM_Port.TOTALLEDCOUNT);
  public final BatteryLED batteryMonitor = new BatteryLED(ledStrip);

  public RobotContainer() {
    configureButtonBindings();
    initShuffleBoard();

    driveBase.setDefaultCommand(new SwerveDriveCommand(this::getXSpeed, 
                                                       this::getYSpeed, 
                                                       this::getRotationSpeed, driveBase));
  }

  public double getXSpeed() { 
    return driver.getLeftY();
  } 

  public double getYSpeed() { 
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

  private SendableChooser<Integer> locationSelector;

  private void initShuffleBoard() {
    locationSelector = new SendableChooser<Integer>();
    locationSelector.setDefaultOption("None", 0);
    locationSelector.addOption("Top Left", 1);
    locationSelector.addOption("Bottom Left", 2);
    locationSelector.addOption("Top Right", 3);
    locationSelector.setDefaultOption("Bottom Right", 4);

    Shuffleboard.getTab("Drive Base").add("Location", locationSelector).withWidget(BuiltInWidgets.kComboBoxChooser);
  }

  private boolean hasSetLocation = false;

  public void setStartingLocation() {
    if(hasSetLocation)
      return;
      
    hasSetLocation = true;
    switch(locationSelector.getSelected()) {
      case 0: /* Do Nothing */ break;
      case 1: driveBase.setStartLocation(1.0, 1.0, 90); break;
      case 2: driveBase.setStartLocation(1.0, 1.0, 0); break;
      case 3: driveBase.setStartLocation(1.0, 1.0, 0); break;
      case 4: driveBase.setStartLocation(1.0, 1.0, 0); break;
    }
  }

  public Command getAutonomousCommand() {
    //Set starting position on autonomous Init
    return null;
  }
}