// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeCargo extends CommandBase {
  /** Creates a new IntakeCargo. */
  public IntakeCargo(Intake intake) {
    this.intake = intake;
    addRequirements(this.intake);
  }

  private Intake intake;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.dropIntake();
    intake.intake();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.liftIntake();
    intake.intakeStop();
  }

}