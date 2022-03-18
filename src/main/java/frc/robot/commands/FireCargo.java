// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class FireCargo extends SequentialCommandGroup {
  public enum Goal {
    Low,
    High
  }
  
  public FireCargo(Shooter shooter, Goal goal) {
    setName("FireCargo");

    addCommands(
      // start the flywheel
      new ConditionalCommand(
         new InstantCommand( shooter::flywheelLowSpeed,  shooter),
         new InstantCommand( shooter::flywheelHighSpeed, shooter),
         () -> { return goal == Goal.Low; } ),
      
      // wait for the flywyeel to get up to speed
      new WaitUntilCommand( shooter::flyWheelReady ),

      // fire until flywheel speed drops
      new RunCommand(shooter::triggerFire, shooter).withInterrupt( () -> { return !shooter.flyWheelReady(); }),

      // stop the trigger
      new InstantCommand(shooter::triggerStop) 
    );
  }
}
