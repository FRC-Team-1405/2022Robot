// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.FilePermission;
import java.lang.reflect.Field;
import java.util.Map;


import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.FireCommand;
import frc.robot.commands.IndexCargo;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.commands.ThreeBallAuto_Inside;
import frc.robot.commands.TwoBallAuto;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.FieldPosition;
//import frc.robot.commands.AutoFireCargo;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.TimedTwoBallAuto;
import frc.robot.commands.DevTestAuto;
import frc.robot.commands.DistanceTwoBallAuto;
import frc.robot.commands.DriveToTest;
import frc.robot.commands.FireAndBackUp;
import frc.robot.commands.AutoFireCargo.Goal;
import frc.robot.sensor.LidarLitePWM;
import frc.robot.commands.FireCargo;
import frc.robot.commands.FireCargoStop;
import frc.robot.commands.IntakeCargo;

public class RobotContainer {
  private final LidarLitePWM lidar = new LidarLitePWM( new DigitalInput(0) );
  private final SwerveDrive driveBase = new SwerveDrive(); 
  private final Shooter shooter = new Shooter(lidar); 
  private final Intake intake = new Intake();
  private final Climber climber = new Climber();
  
  private XboxController driver = new XboxController(Constants.Controller.DRIVER);
  private XboxController operator = new XboxController(Constants.Controller.OPERATOR); 

//  private PowerDistribution pdp = new PowerDistribution(); 

//  private UsbCamera camera = new UsbCamera("Drive Camera", 0);


  public RobotContainer() {
    configureButtonBindings();
    initShuffleBoard(); 
    driveBase.setDefaultCommand(new SwerveDriveCommand(this::getXSpeed, 
                                                       this::getYSpeed, 
                                                       this::getRotationSpeed, driveBase)); 

    climber.setDefaultCommand(new ClimbCommand(this::getLeftClimb, this::getRightClimb, climber));
//    SmartDashboard.putData(pdp);
    
//    camera.setResolution(352, 240);
//    CameraServer.startAutomaticCapture();
  }

  public double getLeftClimb(){
    double speed = -operator.getLeftY();
    if (Math.abs(speed) < 0.3) {
      speed = 0.0;
    }
    return speed;
  }

  public double getRightClimb(){
    double speed = -operator.getRightY();
    if (Math.abs(speed) < 0.3) {
      speed = 0.0;
    } 
    return speed;
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

    // if (Math.abs(driver.getRightX()) <= 0.1)
    //   finalRotation = Math.abs(operator.getRightX()) <= 0.1 ? 0.0 : operator.getRightX() * .5 / (1.0 + operator.getRightTriggerAxis());
    // else
      finalRotation = driver.getRightX() * .5 / (1.0 + driver.getRightTriggerAxis());

      if (Math.abs(finalRotation) < 0.1)
        finalRotation = 0.0;
    
    return finalRotation;
  }


  private void configureButtonBindings() {
    configureDriverButtons();
    configureOperatorButtons();
  }

  private void configureOperatorButtons(){
    // InstantCommand stopCommand = new InstantCommand( shooter::flywheelStop);
    // InstantCommand idleCommand = new InstantCommand( shooter::flywheelIdleSpeed);
    // ConditionalCommand toggleIdleCommand = new ConditionalCommand( idleCommand, stopCommand, shooter::isStopped);

    // new JoystickButton(operator, XboxController.Button.kB.value).whenPressed( toggleIdleCommand );

    Trigger upTrigger = new Trigger( () -> {
      return operator.getPOV() == 0 || operator.getPOV() == 45 || operator.getPOV() == 315;
    });
    Trigger downTrigger = new Trigger( () -> {
      return operator.getPOV() == 180 || operator.getPOV() == 135 || operator.getPOV() == 225;
    });

    Trigger enableClimb = new Trigger( () -> {
      return operator.getStartButton() && operator.getBackButton();
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

    // new JoystickButton(operator, XboxController.Button.kRightBumper.value)
    //     .whenPressed( new InstantCommand(shooter::triggerFire) )
    //     .whenReleased( new InstantCommand(shooter::triggerStop)); 

    new JoystickButton(operator, XboxController.Button.kLeftBumper.value)
        .whenPressed( new InstantCommand(shooter::triggerReverse))
        .whenReleased(new InstantCommand(shooter::triggerStop)); 

    enableClimb.whenActive( climber::enableClimber );
  }   

  private void configureDriverButtons() {
    new JoystickButton(driver, XboxController.Button.kBack.value)
      .whenPressed(new InstantCommand( () -> { driveBase.enableFieldOriented(true); }));

    new JoystickButton(driver, XboxController.Button.kStart.value)
      .whenPressed(new InstantCommand( () -> { driveBase.enableFieldOriented(false);}));

    new JoystickButton(driver, XboxController.Button.kY.value)
        .whileHeld(new FireCargo(shooter, FireCargo.Goal.High))
        .whenReleased( new FireCargoStop(shooter));

    new JoystickButton(driver, XboxController.Button.kA.value)
        .whileHeld(new FireCargo(shooter, FireCargo.Goal.Low))
        .whenReleased(new FireCargoStop(shooter)); 

  new JoystickButton(driver, XboxController.Button.kRightBumper.value)
        .whileHeld(new ParallelCommandGroup(new InstantCommand(intake::dropIntake), new IntakeCargo(intake, shooter)) )
        .whenReleased(new InstantCommand(intake::liftIntake));

  new JoystickButton(driver, XboxController.Button.kLeftBumper.value)
        .whileHeld(new InstantCommand(shooter::triggerFire))
        .whenReleased(new InstantCommand(shooter::triggerStop)); 
}

  private SendableChooser<Integer> locationSelector = new SendableChooser<Integer>(); 
  SendableChooser<Integer> autoSelector = new SendableChooser<Integer>();

  private void initShuffleBoard() {
    locationSelector.addOption("Far Right Tarmac facing Cargo",   0);
    locationSelector.addOption("Left Side of Left Tarmac",        1);
    locationSelector.addOption("Left Tarmac facing Cargo",        2);
    locationSelector.addOption("Right Tarmac facing Cargo",       3);
    locationSelector.setDefaultOption("none",   4);

    Shuffleboard.getTab("Drive Base").add("Location", locationSelector).withWidget(BuiltInWidgets.kComboBoxChooser);
  
    autoSelector.setDefaultOption("Do Nothing",  0); 
    autoSelector.addOption("Shoot and Back Up",  1);
    autoSelector.addOption("Timed 2 Ball",    2);
    autoSelector.addOption("Distance 2 Ball", 3);

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
      case 0: driveBase.setStartLocation(FieldPosition.Tarmac_FarRightReverse); break;
      case 1: driveBase.setStartLocation(FieldPosition.Tarmac_LeftLeft);    break;
      case 2: driveBase.setStartLocation(FieldPosition.Tarmac_LeftReverse); break;
      case 3: driveBase.setStartLocation(FieldPosition.Tarmac_RightReverse);break;
      case 4: break;
      default: driveBase.setStartLocation(FieldPosition.Tarmac_FarRightReverse);    break;
    }

  } 

  public Command getAutonomousCommand() {
    switch (autoSelect()){
      case 0: return new PrintCommand("Do nothing");
      case 1: return new FireAndBackUp(driveBase, intake, shooter, Goal.High);
      case 2: {
        if (   locationSelector.getSelected() == 2
            || locationSelector.getSelected() == 0)
          return new TimedTwoBallAuto(driveBase, intake, shooter, FireCargo.Goal.High, 0.0);
        else 
          return new TimedTwoBallAuto(driveBase, intake, shooter, FireCargo.Goal.High, -30.0);
      }
      case 3: {
        if (   locationSelector.getSelected() == 2
            || locationSelector.getSelected() == 0)
          return new DistanceTwoBallAuto(driveBase, intake, shooter, FireCargo.Goal.High, 0.0);
        else 
          return new DistanceTwoBallAuto(driveBase, intake, shooter, FireCargo.Goal.High, -30.0);
      }
    }

      return new PrintCommand("Default Do Nothing");
    } 
}