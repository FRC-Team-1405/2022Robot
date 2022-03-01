// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;

public class OutTake extends SequentialCommandGroup {
  /** Creates a new OutTake. */
  public Intake intake; 
  public Shooter shooter;
  public OutTake() {
    // Use addRequirements() here to declare subsystem dependencies. 
    addCommands(new InstantCommand(intake::liftIntake, intake), 
                new InstantCommand(shooter::triggerReverse, shooter));

  }
}
