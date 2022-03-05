// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.AutoFireCargo.Goal;
import frc.robot.subsystems.SwerveSubsystem;

public class TwoBallAuto extends SequentialCommandGroup {
  public TwoBallAuto(SwerveSubsystem swerve, Intake intake, Shooter shooter, Goal goal, Pose2d secondCargo) {
    Pose2d cargoPickup = new Pose2d( secondCargo.getX(),
                                     secondCargo.getY(),
                                     Rotation2d.fromDegrees( secondCargo.getRotation().getDegrees()+180.0) );

    addCommands(new AutoFireCargo(intake, shooter, goal), 
                new ParallelRaceGroup(new SequentialCommandGroup( new TurnToAngle( cargoPickup.getRotation().getDegrees(), swerve),
                                                                  new RunPath( List.of(cargoPickup), swerve) ), 
                                      new IntakeCargo(intake, shooter) ), 
                new TurnToAngle( swerve.getPose().getRotation().getDegrees(), swerve),
                new RunPath( List.of(swerve.getPose()), swerve), 
                new AutoFireCargo(intake, shooter, goal)); 
  }
}
