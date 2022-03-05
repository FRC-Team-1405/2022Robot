package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FieldPosition;
import frc.robot.commands.AutoFireCargo.Goal;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;

public class ThreeBallAuto_Inside extends SequentialCommandGroup {
  public ThreeBallAuto_Inside(SwerveSubsystem swerve, Shooter shooter, Intake intake, Goal goal, Pose2d secondCargo, Pose2d secondFire, Pose2d thirdCargo, Pose2d thirdFire) {
    Pose2d secondPickup = new Pose2d( secondCargo.getX(),
                                      secondCargo.getY(),
                                      Rotation2d.fromDegrees( secondCargo.getRotation().getDegrees()+180.0) );
    Pose2d thirdPickup  = new Pose2d( thirdCargo.getX(),
                                      thirdCargo.getY(),
                                      Rotation2d.fromDegrees( thirdCargo.getRotation().getDegrees()+180.0) );

    addCommands(new AutoFireCargo(intake, shooter, goal), 
                new ParallelRaceGroup(new SequentialCommandGroup( new TurnToAngle( secondPickup.getRotation().getDegrees(), swerve),
                                                                  new RunPath( List.of(secondPickup), swerve) ), 
                                      new IntakeCargo(intake, shooter) ), 
                new TurnToAngle( secondFire.getRotation().getDegrees(), swerve),
                new RunPath( List.of(secondFire), swerve), 
                new AutoFireCargo(intake, shooter, goal),
                new ParallelRaceGroup(new SequentialCommandGroup( new TurnToAngle( thirdPickup.getRotation().getDegrees(), swerve),
                                                                  new RunPath( List.of(thirdPickup), swerve) ), 
                                      new IntakeCargo(intake, shooter) ), 
                new TurnToAngle( thirdFire.getRotation().getDegrees(), swerve),
                new RunPath( List.of(thirdFire), swerve), 
                new AutoFireCargo(intake, shooter, goal)
                ); 
  }
}
