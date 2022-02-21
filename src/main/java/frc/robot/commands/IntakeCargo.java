// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeCargo extends CommandBase {
  public IntakeCargo(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  private Intake intake;

  @Override
  public void initialize() {
    //intake.dropIntake();
    intake.intake();
  }

  @Override
  public void end(boolean interrupted) {
    //intake.liftIntake();
    intake.intakeStop();
  }
}
