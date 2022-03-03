// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Shooter;

public class IndexCargo extends SequentialCommandGroup {
  /** Creates a new IndexCargo. */
  public IndexCargo(Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    addCommands( new InstantCommand(shooter::indexReverse, shooter),
                 new WaitCommand(.25),
                 new InstantCommand(shooter::indexStop, shooter) ); 
  }
}
