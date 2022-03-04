package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoFireCargo.Goal;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;

public class DevTestAuto extends SequentialCommandGroup {
    public DevTestAuto(SwerveSubsystem swerve, Shooter shooter, Goal goal) {
        Pose2d startPose = swerve.getPose();
        Pose2d target = new Pose2d(startPose.getX()-1, startPose.getY(), Rotation2d.fromDegrees(startPose.getRotation().getDegrees()+180));
        addRequirements(swerve);

        addCommands( new TurnToAngle( startPose.getRotation().getDegrees()+180, swerve),
                     new RunPath( List.of( target ), swerve )
                    );
    }
}
  