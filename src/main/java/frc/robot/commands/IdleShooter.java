// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Shooter;

public class IdleShooter extends CommandBase {
  /** Creates a new IdleShooter. */ 
  private Shooter shooter; 
  private Goal goal;
  public enum Goal {
    Low,
    High, 
    Off
  }
  
  public IdleShooter(Shooter shooter, Goal goal) {
    // Use addRequirements() here to declare subsystem dependencies. 
    this.shooter = shooter; 
    this.goal = goal; 
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() { 
    if(goal == Goal.High){ 
      shooter.flywheelHighSpeed();
    } else if (goal == Goal.Low){ 
      shooter.flywheelLowSpeed();
    } else { 
      shooter.flywheelStop();
    }
}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {  
      }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
