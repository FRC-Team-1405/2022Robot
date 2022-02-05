// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  TalonSRX flyWheel = new WPI_TalonSRX(Constants.CAN_ID.FLYWHEEL);
  /** Creates a new Shooter. */
  public Shooter() {
    Preferences.initInt("Shooter/Speed/Low", lowSpeed);
    lowSpeed = Preferences.getInt("Shooter/Speed/Low", lowSpeed);
    Preferences.initInt("Shooter/Speed/High", highSpeed);
    highSpeed = Preferences.getInt("Shooter/Speed/High", highSpeed);
    Preferences.initInt("Shooter/Speed/Idle", idleSpeed);
    idleSpeed = Preferences.getInt("Shooter/Speed/Idle", idleSpeed);
    Preferences.initInt("Shooter/Speed/Adjust", adjustSpeed);
    adjustSpeed = Preferences.getInt("Shooter/Speed/Adjust", adjustSpeed);

    setLowIndex(speedLowIndex);
    setHighIndex(speedHighIndex);
  }
  private int speedLowIndex = 0;
  private int speedHighIndex = 0;
  private int adjustSpeed = 2500;
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
    flywheelSpeed( getLowSpeed() );
  }

  public int getLowSpeed(){
    return lowSpeed + (adjustSpeed * speedLowIndex);
  }
  public int getHighSpeed(){
    return highSpeed + (adjustSpeed * speedHighIndex);
  }

  public void flywheelHighSpeed(){
    flywheelSpeed(highSpeed);
  }
  public void flywheelIdleSpeed(){
    flywheelSpeed(idleSpeed);
  }

  public void increaseLowIndex(){
    setLowIndex(speedLowIndex+1);
  }

  public void decreaseLowIndex(){
    setLowIndex(speedLowIndex-1);
  }

  private void setLowIndex(int value){
    if (value > 3){
      value = 3;
    } else if (value < -3){
      value = -3;
    }

    speedLowIndex = value;
    SmartDashboard.putNumber("Shooter/Speed/Low", getLowSpeed() );
  }

  public void increaseHighIndex(){
    setHighIndex(speedHighIndex+1);
  }

  public void decreaseHighIndex(){
    setHighIndex(speedHighIndex-1);
  }

  private void setHighIndex(int value){
    if (value > 3){
      value = 3;
    } else if (value < -3){
      value = -3;
    }

    speedHighIndex = value;
    SmartDashboard.putNumber("Shooter/Speed/High", getHighSpeed() );
  }
  
  public boolean isStopped(){
    return flyWheel.getControlMode() == ControlMode.PercentOutput;
  }

  private void flywheelSpeed(int speed){
    if (speed == 0){
      flyWheel.set(ControlMode.PercentOutput, 0);
    } else {
      flyWheel.set(ControlMode.Velocity, speed);
    }
  }
}
