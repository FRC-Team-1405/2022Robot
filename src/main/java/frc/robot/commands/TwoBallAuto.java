// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.AutoFireCargo.Goal;
import frc.robot.subsystems.SwerveDrive;

public class TwoBallAuto extends SequentialCommandGroup {
  /** Creates a new TwoBallAuto. */
  public TwoBallAuto(SwerveDrive swerveDrive, Shooter shooter, Intake intake, Goal goal) {
    // Use addRequirements() here to declare subsystem dependencies. 
    addCommands(new AutoFireCargo(shooter, goal), 
               // new ParallelRaceGroup(new RunTrajectory(trajectory, swerveDrive), new IntakeCargo(intake)), 
              //  new ParallelCommandGroup(new RunTrajectory(trajectory, swerveDrive), new IdleShooter()), 
                new AutoFireCargo(shooter, goal)); 
  }
}