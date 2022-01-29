// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  WPI_TalonSRX flyWheel = new WPI_TalonSRX(Constants.CAN_ID.FLYWHEEL);
  /** Creates a new Shooter. */
  public Shooter() {
    Preferences.initInt("Shooter/Speed/Low", lowSpeed);
    lowSpeed = Preferences.getInt("Shooter/Speed/Low", lowSpeed);
    Preferences.initInt("Shooter/Speed/High", highSpeed);
    highSpeed = Preferences.getInt("Shooter/Speed/High", highSpeed);
    Preferences.initInt("Shooter/Spee/Idle", idleSpeed);
  }
  private int idleSpeed = 20000;
  private int lowSpeed = 20000;
  private int highSpeed =30000;
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void flywheelStop(){
    flywheelSpeed(0);
  }

  public void flywheelLowSpeed(){
    flywheelSpeed(lowSpeed);
  }

  public void flywheelHighSpeed(){
    flywheelSpeed(highSpeed);
  }
  public void flywheelIdleSpeed(){
    flywheelSpeed(idleSpeed);
  }

  private void flywheelSpeed(int speed){
    flyWheel.set(ControlMode.Velocity, speed);
  }
}
