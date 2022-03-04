package frc.robot.commands;

import java.util.List;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.trajectories.GenerateTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;

public class RunPath extends CommandBase{

    private int currentPosition;
    private final List<Pose2d> waypoints;
    private RunTrajectory runTrajectory;
    private SwerveSubsystem swerve;

    public RunPath(List<Pose2d> waypoints, SwerveSubsystem swerve){
        this.waypoints = waypoints;
        this.swerve = swerve;
    }

    public void initialize() {
        setupWaypoint(0);
    }

    public void execute() {
        runTrajectory.execute();
    }

    public void end(boolean interrupted) {
        swerve.drive(0, 0, 0);
    }

    public boolean isFinished() {
        if (runTrajectory.isFinished()){
            if (currentPosition == waypoints.size()-1)
                return true;
            else
                setupWaypoint(currentPosition+1);
        }
        return false;
    }

    private void setupWaypoint(int index){
        currentPosition = index;
        Trajectory trajectory =  GenerateTrajectory.driveTo(swerve.getPose(), waypoints.get(currentPosition), swerve);

        runTrajectory = new RunTrajectory( trajectory, 
                                           () -> { return waypoints.get(currentPosition).getRotation(); }, 
                                           swerve);
        runTrajectory.initialize();
    }
    
}
