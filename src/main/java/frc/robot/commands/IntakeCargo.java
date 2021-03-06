// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class IntakeCargo extends CommandBase {
  /** Creates a new IntakeCargo. */
  public IntakeCargo(Intake intake, Shooter shooter) {
    this.intake = intake;
    this.shooter = shooter;
    addRequirements(this.intake);
    addRequirements(this.shooter);
  }

  private Intake intake;
  private Shooter shooter;

  @Override
  public void initialize() {
    //intake.dropIntake();
    intake.intake();
    shooter.triggerFire();
  }

  @Override
  public void end(boolean interrupted) {
    //intake.liftIntake();
    intake.intakeStop();
    shooter.triggerStop();
  }
}
