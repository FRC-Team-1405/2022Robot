// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoFireCargo.Goal;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.trajectories.GenerateTrajectory;

public class FireAndBackUp extends SequentialCommandGroup {
  public FireAndBackUp(SwerveDrive swerveDrive, Shooter shooter, Goal goal) {
    addCommands( new AutoFireCargo(shooter, goal),
                 new RunTrajectory(GenerateTrajectory.backupOneMeter(swerveDrive.getPose(), swerveDrive), swerveDrive));;
  }
}
