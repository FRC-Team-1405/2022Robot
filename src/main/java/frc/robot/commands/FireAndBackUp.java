// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoFireCargo.Goal;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.trajectories.GenerateTrajectory;

public class FireAndBackUp extends SequentialCommandGroup {
  public FireAndBackUp(SwerveDrive swerveDrive, Intake intake, Shooter shooter, Goal goal) {
    addRequirements(swerveDrive);
    Pose2d targetPos = swerveDrive.getPose().relativeTo( new Pose2d(-1,0, swerveDrive.getPose().getRotation()) );
    addCommands(  new InstantCommand( intake::dropIntake ),
                  new AutoFireCargo(intake, shooter, goal),
                  // new RunPath(List.of(targetPos), swerveDrive),
                  new RunCommand(() -> {swerveDrive.drive(-0.25, 0.0, 0.0);}, swerveDrive).withTimeout(1.0)
                  );

    }
}
