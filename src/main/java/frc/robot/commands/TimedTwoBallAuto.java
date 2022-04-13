// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.FireCargo.Goal;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveSubsystem;

public class TimedTwoBallAuto extends SequentialCommandGroup {
  /** Creates a new DeadReckonTwoBallAuto. */
  public TimedTwoBallAuto(SwerveDrive swerve, Intake intake, Shooter shooter, Goal goal, double angleAdjust) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
    addRequirements(shooter);
    addRequirements(intake); 
    addCommands( new InstantCommand( intake::dropIntake ),
                 new InstantCommand(() -> {swerve.resetDistance();}),
                 new ParallelRaceGroup(
                      new IntakeCargo(intake, shooter),
                      new RunCommand(() -> {
                        swerve.driveSpeed(0.8, 0.0, 0.0, false);
                        SmartDashboard.putNumber("Auto/Distance", Units.metersToInches(swerve.getDistance()));
                       }, swerve).withTimeout(2.5)),
                 new IndexCargo(shooter),
                 new InstantCommand(() -> {swerve.resetDistance();}),
                 new RunCommand(() -> {
                   swerve.driveSpeed(-0.8, 0.0, 0.0, false);
                   SmartDashboard.putNumber("Auto/Distance", Units.metersToInches(swerve.getDistance()));
                  }, swerve).withTimeout(3.5),
             new TurnToAngle( swerve.getPose().getRotation().getDegrees()+180+angleAdjust, swerve),
                 new FireCargo(shooter, goal)
                );

  }

}
