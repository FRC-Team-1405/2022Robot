// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.reflect.Field;
import java.util.Map;


import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.FireCommand;
import frc.robot.commands.IndexCargo;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.commands.ThreeBallAuto_Inside;
import frc.robot.commands.TwoBallAuto;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.FieldPosition;
import frc.robot.commands.AutoFireCargo;
import frc.robot.commands.DevTestAuto;
import frc.robot.commands.DriveToTest;
import frc.robot.commands.FireAndBackUp;
import frc.robot.commands.AutoFireCargo.Goal;
import frc.robot.commands.FireCargo;
import frc.robot.commands.FireCargoStop;
import frc.robot.commands.IntakeCargo;

public class RobotContainer {
  private final SwerveDrive driveBase = new SwerveDrive(); 
  private final Shooter shooter = new Shooter(); 
  private final Intake intake = new Intake();
  
  private XboxController driver = new XboxController(Constants.Controller.DRIVER);
  private XboxController operator = new XboxController(Constants.Controller.OPERATOR);

//  private UsbCamera camera = new UsbCamera("Drive Camera", 0);


  public RobotContainer() {
    configureButtonBindings();
    initShuffleBoard();
    driveBase.setDefaultCommand(new SwerveDriveCommand(this::getXSpeed, 
                                                       this::getYSpeed, 
                                                       this::getRotationSpeed, driveBase));
    
//    camera.setResolution(352, 240);
//    CameraServer.startAutomaticCapture();
  }

  public double getXSpeed(){ 
    double finalX;
    if (Math.abs(driver.getLeftY()) <= 0.1)
      finalX = 0.0;
    else
      finalX = driver.getLeftY() * 0.5 * (1.0 + driver.getLeftTriggerAxis());
    
    return -finalX;
  } 

  public double getYSpeed(){ 
    double finalY;
    if (Math.abs(driver.getLeftX()) <= 0.1)
      finalY = 0.0;
    else
      finalY = driver.getLeftX() * 0.5 * (1.0 + driver.getLeftTriggerAxis());
    
    return finalY;
  } 

  public double getRotationSpeed(){ 
    double finalRotation;
    if (Math.abs(driver.getRightX()) <= 0.1) 
      finalRotation = Math.abs(operator.getRightX()) <= 0.1 ? 0.0 : operator.getRightX() * .5 / (1.0 + operator.getRightTriggerAxis());
    else
      finalRotation = driver.getRightX() * .5 / (1.0 + driver.getRightTriggerAxis());

    
    return finalRotation;
  }


  private void configureButtonBindings() {
    configureDriverButtons();
    configureOperatorButtons();
  }

  private void configureOperatorButtons(){
    InstantCommand stopCommand = new InstantCommand( shooter::flywheelStop);
    InstantCommand idleCommand = new InstantCommand( shooter::flywheelIdleSpeed);
    ConditionalCommand toggleIdleCommand = new ConditionalCommand( idleCommand, stopCommand, shooter::isStopped);

    new JoystickButton(operator, XboxController.Button.kB.value).whenPressed( toggleIdleCommand );

    Trigger upTrigger = new Trigger( () -> {
      return operator.getPOV() == 0 || operator.getPOV() == 45 || operator.getPOV() == 315;
    });
    Trigger downTrigger = new Trigger( () -> {
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

    new JoystickButton(operator, XboxController.Button.kRightBumper.value)
        .whenPressed( new InstantCommand(shooter::triggerFire) )
        .whenReleased( new InstantCommand(shooter::triggerStop)); 

    new JoystickButton(operator, XboxController.Button.kLeftBumper.value)
        .whenPressed( new InstantCommand(shooter::triggerReverse))
        .whenReleased(new InstantCommand(shooter::triggerStop)); 
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
        .whileHeld(new IntakeCargo(intake, shooter))
        .whenReleased(new IndexCargo(shooter));

  new JoystickButton(driver, XboxController.Button.kLeftBumper.value)
        .whenPressed(new InstantCommand(intake::deployRetractIntake, intake));
}

  private SendableChooser<Integer> locationSelector = new SendableChooser<Integer>(); 
  SendableChooser<Integer> autoSelector = new SendableChooser<Integer>();

  private void initShuffleBoard() {
    locationSelector.setDefaultOption("None",                 0);
    locationSelector.addOption("Left Side of Left Tarmac",    1);
    locationSelector.addOption("Center of Left Tarmac",       2);
    locationSelector.addOption("Right Side of Left Tarac",    3);
    locationSelector.addOption("Left Side of Right Tarmac",   4);
    locationSelector.addOption("Center of Right Tarmac",      5);
    locationSelector.addOption("Right Side of Right Tarmac",  6);

    Shuffleboard.getTab("Drive Base").add("Location", locationSelector).withWidget(BuiltInWidgets.kComboBoxChooser);
  
    autoSelector.setDefaultOption("Do Nothing", 0); 
    autoSelector.addOption("Shoot Only",        1); 
    autoSelector.addOption("Shoot and Back Up", 2);
    autoSelector.addOption("2 Ball Auto",       3);
    autoSelector.addOption("3 Ball Auto",       4);

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
      case 1: driveBase.setStartLocation(FieldPosition.Tarmac_LeftLeft);    break;
      case 2: driveBase.setStartLocation(FieldPosition.Tarmac_LeftCenter);  break;
      case 3: driveBase.setStartLocation(FieldPosition.Tarmac_LeftRight);   break;
      case 4: driveBase.setStartLocation(FieldPosition.Tarmac_RightLeft);   break;
      case 5: driveBase.setStartLocation(FieldPosition.Tarmac_RightCenter); break;
      case 6: driveBase.setStartLocation(FieldPosition.Tarmac_RightRight);  break;
      default: /* Do Nothing */ break;
    }

  } 

  public Command getAutonomousCommand() {
    switch (autoSelect()){
      case 0: return new PrintCommand("Do nothing");
      case 1: return new AutoFireCargo(intake, shooter, Goal.High); 
      case 2: return new FireAndBackUp(driveBase, intake, shooter, Goal.High);
      case 3: {
        if (   locationSelector.getSelected() == 1
            || locationSelector.getSelected() == 2
            || locationSelector.getSelected() == 3)
          return new TwoBallAuto(driveBase,  intake, shooter,Goal.High, FieldPosition.Cargo_Left);
        else if (    locationSelector.getSelected() == 4
                  || locationSelector.getSelected() == 5
                  || locationSelector.getSelected() == 6)
          return new TwoBallAuto(driveBase, intake, shooter, Goal.High, FieldPosition.Cargo_Center);
      }
      case 4: {
        if (   locationSelector.getSelected() == 1
            || locationSelector.getSelected() == 2
            || locationSelector.getSelected() == 3)
          return new  ThreeBallAuto_Inside( driveBase, shooter, intake, Goal.High, 
                                            FieldPosition.Cargo_Center, FieldPosition.Tarmac_LeftLeft, 
                                            FieldPosition.Cargo_Left,   FieldPosition.Tarmac_RightCenter);
        else if (    locationSelector.getSelected() == 4
                  || locationSelector.getSelected() == 5
                  || locationSelector.getSelected() == 6)
          return new  ThreeBallAuto_Inside( driveBase, shooter, intake, Goal.High, 
                                            FieldPosition.Cargo_Left,   FieldPosition.Tarmac_RightCenter, 
                                            FieldPosition.Cargo_Center,  FieldPosition.Tarmac_LeftLeft);
      }
    }

      return new PrintCommand("Default Do Nothing");
    } 
}