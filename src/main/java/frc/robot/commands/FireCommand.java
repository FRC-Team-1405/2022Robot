// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class FireCommand extends CommandBase {
  private Shooter shooter; 
  private Timer timer = new Timer();
  
  public FireCommand(Shooter shooter) {
    addRequirements(shooter);
    this.shooter = shooter;
  }

  @Override
  public void initialize() {
    shooter.flywheelHighSpeed();
  }

  @Override
  public void execute() {
    if (shooter.flyWheelReady()) {
      shooter.triggerFire(); 
      timer.start();
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.flywheelStop(); 
    shooter.triggerStop();
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(1);
  }
}
