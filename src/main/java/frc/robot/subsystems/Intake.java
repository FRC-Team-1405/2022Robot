// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  public WPI_TalonSRX pickupMotor = new WPI_TalonSRX(Constants.CAN_ID.PICKUP); 
  public WPI_TalonSRX intakeDropper = new WPI_TalonSRX(Constants.CAN_ID.INTAKE_DROPPER); 

  public double INTAKE_UP = -0.5; 
  public double INTAKE_DOWN = 0.35; 

  public Intake() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  } 

  public void dropIntake(){ 
    intakeDropper.set(ControlMode.PercentOutput, INTAKE_DOWN); 
    intake();
  } 

  public void liftIntake(){ 
    intakeDropper.set(ControlMode.PercentOutput, INTAKE_UP); 
    intakeStop();
  } 

  public void intake(){ 
    pickupMotor.set(ControlMode.PercentOutput, 0.6);
  } 

  public void intakeStop(){ 
    pickupMotor.set(ControlMode.PercentOutput, 0.0);
  }
}
