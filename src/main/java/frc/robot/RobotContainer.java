// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoFireCargo;
import frc.robot.commands.AutoFireCargo.Goal;
import frc.robot.commands.BatteryLED;
import frc.robot.commands.FireCargo;
import frc.robot.commands.FireCargoStop;
import frc.robot.commands.FireCommand;
import frc.robot.commands.IntakeCargo;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.sensor.LEDStrip;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive;

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

  public double getXSpeed(){ 
    double finalX;
    if (Math.abs(driver.getLeftY()) <= 0.1)
      finalX = 0.0;
    else
      finalX = driver.getLeftY() * 0.5 * (1.0 + driver.getLeftTriggerAxis());
    
    SmartDashboard.putNumber("xSpeed", finalX);
    return finalX;
  } 

  public double getYSpeed(){ 
    double finalY;
    if (Math.abs(driver.getLeftX()) <= 0.1)
      finalY = 0.0;
    else
      finalY = driver.getLeftX() * 0.5 * (1.0 + driver.getLeftTriggerAxis());
    
    SmartDashboard.putNumber("ySpeed", finalY);
    return finalY;
  } 

  public double getRotationSpeed(){ 
    double finalRotation;
    if (Math.abs(driver.getRightX()) <= 0.1)
      finalRotation = 0.0;
    else
      finalRotation = driver.getRightX() * 0.5 * (1.0 + driver.getLeftTriggerAxis());

    SmartDashboard.putNumber("rotationSpeed", finalRotation);
    return finalRotation;
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
    new JoystickButton(driver, XboxController.Button.kBack.value)
      .whenPressed(new InstantCommand( () -> { driveBase.enableFieldOriented(true); }));

    new JoystickButton(driver, XboxController.Button.kStart.value)
      .whenPressed(new InstantCommand( () -> { driveBase.enableFieldOriented(false);}));

  new JoystickButton(driver, XboxController.Button.kY.value)
        .whileHeld( new FireCargo(shooter, FireCargo.Goal.High) )
        .whenReleased( new FireCargoStop(shooter));

  new JoystickButton(driver, XboxController.Button.kA.value)
        .whileHeld( new FireCargo(shooter, FireCargo.Goal.Low) )
        .whenReleased(new FireCargoStop(shooter)); 
  
  

  new JoystickButton(driver, XboxController.Button.kRightBumper.value)
        .toggleWhenPressed( new IntakeCargo(intake) );

}

  private SendableChooser<Integer> locationSelector; 
  SendableChooser<Integer> autoSelector;

  private void initShuffleBoard() {
    locationSelector = new SendableChooser<Integer>();
    locationSelector.setDefaultOption("None", 0);
    locationSelector.addOption("Top Left", 1);
    locationSelector.addOption("Bottom Left", 2);
    locationSelector.addOption("Top Right", 3);
    locationSelector.addOption("Bottom Right", 4);

    Shuffleboard.getTab("Drive Base").add("Location", locationSelector).withWidget(BuiltInWidgets.kComboBoxChooser);
  
    autoSelector = new SendableChooser<Integer>();
    autoSelector.setDefaultOption("Do Nothing", 0); 
    autoSelector.addOption("Shoot Only", 1); 
    autoSelector.addOption("Shoot Only Refactor", 2);

    Shuffleboard.getTab("Auto").add("Auto", autoSelector).withWidget(BuiltInWidgets.kComboBoxChooser);
  }

  private int autoSelect() { 
    return (int) autoSelector.getSelected();
  }

  private boolean hasSetLocation = false;

  public void setStartingLocation() {
    if(hasSetLocation)
      return;
      
    hasSetLocation = true;
    switch(locationSelector.getSelected()) {
      case 0: /* Do Nothing */ break;
      case 1: driveBase.setStartLocation(1.0, 1.0, 90); break;
      case 2: driveBase.setStartLocation(1.0, 1.0, 45); break;
      case 3: driveBase.setStartLocation(1.0, 1.0, 180); break;
      case 4: driveBase.setStartLocation(1.0, 1.0, -90); break;
    }
  } 

  private Command shootOnlyAuto(){
    return new FireCommand(shooter);
  } 

  private Command autoFireCargo(){ 
    return new AutoFireCargo(shooter, Goal.High);
  }

  private final Command selectCommand =
  new SelectCommand(
      // Maps selector values to commands
      Map.ofEntries(
          Map.entry(0, new PrintCommand("Do nothing")),
          Map.entry(1, shootOnlyAuto()), 
          Map.entry(2, autoFireCargo())
       ),
      this::autoSelect
  );

    public Command getAutonomousCommand() {
      // return autoCommand;
      return selectCommand;
    } 
}