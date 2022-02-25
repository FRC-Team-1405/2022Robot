package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoFireCargo.Goal;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.trajectories.GenerateTrajectory;

public class DevTestAuto extends SequentialCommandGroup {
    public DevTestAuto(SwerveDrive swerveDrive, Shooter shooter, Goal goal) {
        Pose2d currentPose = swerveDrive.getPose();
        Trajectory turnAndDrive = GenerateTrajectory.driveTo(currentPose, new Pose2d(-3,0, new Rotation2d()));
        addCommands( new RunTrajectory(turnAndDrive, swerveDrive) );
    }
}
  