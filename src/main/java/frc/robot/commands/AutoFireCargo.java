// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class AutoFireCargo extends SequentialCommandGroup {
  public enum Goal {
    Low,
    High
  }

  private Intake intake;
  private Shooter shooter;

  public AutoFireCargo(Intake intake, Shooter shooter, Goal goal) {
    this.intake = intake;
    this.shooter = shooter;

    setName("AutoFireCargo");

    addCommands(
      new IndexCargo(shooter),
      // start the flywheel
      new ConditionalCommand(
         new InstantCommand( shooter::flywheelLowSpeed,  shooter),
         new InstantCommand( shooter::flywheelHighSpeed, shooter),
         () -> { return goal == Goal.Low; } ),
      
      // wait for the flywyeel to get up to speed
      new WaitUntilCommand( shooter::flyWheelReady ),

      // fire until flywheel speed drops
      new RunCommand( () -> {shooter.triggerFire(); intake.intake();}, shooter)
        .withInterrupt( () -> { return !shooter.flyWheelReady(); }),
        // .withTimeout(5),

      // stop trigger and flywheel
      new InstantCommand( () -> {
        shooter.triggerStop();
        intake.intakeStop();
      },
      shooter),

      // add a small wait
      new WaitCommand( 0.5 ) ,

      // wait for the flywyeel to get up to speed
      new WaitUntilCommand( shooter::flyWheelReady ),

      // fire until flywheel speed drops
      new RunCommand( () -> {shooter.triggerFire(); intake.intake();}, shooter)
        .withInterrupt( () -> { return !shooter.flyWheelReady(); }),
        // .withTimeout(6),

      // stop trigger and flywheel
      new InstantCommand( () -> {
          shooter.triggerStop();
          shooter.flywheelStop();
          intake.intakeStop();
        },
        shooter)
    );  
  }
  public void end(boolean interrupted) {
    shooter.triggerStop();
    shooter.flywheelStop();
    intake.intakeStop();
  }
}
