package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FieldPosition;
import frc.robot.commands.AutoFireCargo.Goal;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;

public class ThreeBallAuto_Inside extends SequentialCommandGroup {
  public ThreeBallAuto_Inside(SwerveSubsystem swerve, Shooter shooter, Intake intake, Goal goal, Pose2d secondCargo, Pose2d secondFire, Pose2d thridCargo, Pose2d thirdFire) {
    addCommands(new AutoFireCargo(shooter, goal), 
                new ParallelRaceGroup(new SequentialCommandGroup( new TurnToAngle( secondCargo.getRotation().getDegrees()+180.0, swerve),
                                                                  new RunPath( List.of(secondCargo), swerve) ), 
                                      new IntakeCargo(intake, shooter) ), 
                new TurnToAngle( secondFire.getRotation().getDegrees(), swerve),
                new RunPath( List.of(secondFire), swerve), 
                new AutoFireCargo(shooter, goal),
                new ParallelRaceGroup(new RunPath( List.of(secondCargo), swerve), 
                                      new IntakeCargo(intake, shooter) ), 
                new TurnToAngle( thridCargo.getRotation().getDegrees(), swerve),
                new RunPath( List.of(thirdFire), swerve), 
                new AutoFireCargo(shooter, goal)
                ); 
  }
}
