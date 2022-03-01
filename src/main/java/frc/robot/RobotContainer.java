// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.FireCommand;
import frc.robot.commands.IndexCargo;
import frc.robot.commands.SwerveDriveCommand;
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
import frc.robot.commands.AutoFireCargo;
import frc.robot.commands.FireAndBackUp;
import frc.robot.commands.AutoFireCargo.Goal;
import frc.robot.commands.FireCargo;
import frc.robot.commands.FireCargoStop;
import frc.robot.commands.IntakeCargo;
import frc.robot.commands.OutTake;

public class RobotContainer {
  private final SwerveDrive driveBase = new SwerveDrive(); 
  private final Shooter shooter = new Shooter(); 
  private final Intake intake = new Intake();
  
  private XboxController driver = new XboxController(Constants.Controller.DRIVER);
  private XboxController operator = new XboxController(Constants.Controller.OPERATOR);

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
      finalRotation = 0.0;
    else
      finalRotation = driver.getRightX() / (1.0 + driver.getRightTriggerAxis());

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
        .whenReleased( new SequentialCommandGroup( new InstantCommand(shooter::indexReverse, shooter),
                                                   new WaitCommand(0.1),
                                                   new InstantCommand(shooter::indexStop, shooter))); 

    new JoystickButton(operator, XboxController.Button.kLeftBumper.value)
        .whenPressed( new OutTake());
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
    locationSelector.setDefaultOption("None", 0);
    locationSelector.addOption("0 Top Left", 1);
    locationSelector.addOption("45 Bottom Left", 2);
    locationSelector.addOption("-45 Top Right", 3);
    locationSelector.addOption("Bottom Right", 4);

    Shuffleboard.getTab("Drive Base").add("Location", locationSelector).withWidget(BuiltInWidgets.kComboBoxChooser);
  
    autoSelector.setDefaultOption("Do Nothing", 0); 
    autoSelector.addOption("Shoot Only", 1); 
    autoSelector.addOption("Shoot Only Refactor", 2); 
    autoSelector.addOption("Shoot and Back Up", 3);

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
      case 1: driveBase.setStartLocation(1.0, 1.0, 0); break;
      case 2: driveBase.setStartLocation(1.0, 1.0, 45); break;
      case 3: driveBase.setStartLocation(1.0, 1.0, -45); break;
      case 4: driveBase.setStartLocation(1.0, 1.0, -90); break;
    }

  } 

  private final Command selectCommand =
  new SelectCommand(
      // Maps selector values to commands
      Map.ofEntries(
          Map.entry(0, new PrintCommand("Do nothing")),
          Map.entry(1, new FireCommand(shooter)), 
          Map.entry(2, new AutoFireCargo(shooter, Goal.High)), 
          Map.entry(3, new FireAndBackUp(driveBase, shooter, Goal.Low))
       ),
      this::autoSelect
  );

    public Command getAutonomousCommand() {
      switch (autoSelect()){
        case 0: return new PrintCommand("Do nothing");
        case 1: return new FireCommand(shooter); 
        case 2: return new AutoFireCargo(shooter, Goal.High); 
        case 3: return new FireAndBackUp(driveBase, shooter, Goal.High);
      }
      // return autoCommand;
      return selectCommand;
    } 
}