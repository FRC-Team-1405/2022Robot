// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.sensor.LidarLitePWM;
import frc.robot.sensor.UltrasonicSensor;

public class Shooter extends SubsystemBase {
  TalonFX flyWheel = new WPI_TalonFX(Constants.CANID.FLYWHEEL);
  WPI_TalonSRX trigger = new WPI_TalonSRX(Constants.CANID.TRIGGER); 
  UltrasonicSensor ultrasonicSensor = new UltrasonicSensor(Constants.Sensors.ULTRASONICSENSOR);
  LidarLitePWM lidar;

  public Shooter(LidarLitePWM lidar) {
    this.lidar = lidar;
    
    Preferences.initInt("Shooter/Speed/Low", lowSpeed);
    lowSpeed = Preferences.getInt("Shooter/Speed/Low", lowSpeed);
    Preferences.initInt("Shooter/Speed/High", highSpeed);
    highSpeed = Preferences.getInt("Shooter/Speed/High", highSpeed);
    Preferences.initInt("Shooter/Speed/Idle", idleSpeed);
    idleSpeed = Preferences.getInt("Shooter/Speed/Idle", idleSpeed);
    Preferences.initInt("Shooter/Speed/Adjust", adjustSpeed);
    adjustSpeed = Preferences.getInt("Shooter/Speed/Adjust", adjustSpeed);
    Preferences.initDouble("Shooter/Trigger/Speed", triggerSpeed);
    triggerSpeed = Preferences.getDouble("Shooter/Trigger/Speed", triggerSpeed);

    Preferences.initDouble("Shooter/Distance/Close", distanceClose);
    distanceClose = Preferences.getDouble("Shooter/Distance/Close", distanceClose);
    Preferences.initDouble("Shooter/Distance/Far", distanceFar);
    distanceClose = Preferences.getDouble("Shooter/Distance/Far", distanceClose);

    setLowIndex(speedLowIndex);
    setHighIndex(speedHighIndex);
  }

  private double triggerSpeed = Constants.Shooter.INDEX_SPEED;
  private int speedLowIndex = 0;
  private int speedHighIndex = 0;
  private int adjustSpeed = 1000;
  private int idleSpeed = 5000;
  private int lowSpeed = 5000;
  private int highSpeed = 8000;

  private double distanceClose = 100.0;
  private double distanceFar   = 200.0;

  @Override
  public void periodic() {
    double distance = lidar.getDistance();

    SmartDashboard.putNumber("Shooter/Flywheel Error", Math.abs(flyWheel.getClosedLoopError())/500);
    SmartDashboard.putBoolean("Shooter/Distance/Close", distance < distanceClose);
    SmartDashboard.putBoolean("Shooter/Distance/Far", distance > distanceFar);
  }

  public void flywheelStop() {
    flywheelSpeed(0);
  }

  public void flywheelLowSpeed() {
    flywheelSpeed(getLowSpeed());
  }

  public int getLowSpeed() {
    return lowSpeed + (adjustSpeed * speedLowIndex);
  }

  public int getHighSpeed() {
    return highSpeed + (adjustSpeed * speedHighIndex);
  }

  public void flywheelHighSpeed() {
    flywheelSpeed(getHighSpeed());
  }

  public void flywheelIdleSpeed() {
    flywheelSpeed(idleSpeed);
  }

  public void increaseLowIndex() {
    setLowIndex(speedLowIndex + 1);
  }

  public void decreaseLowIndex() {
    setLowIndex(speedLowIndex - 1);
  }

  public void triggerFire() {
    trigger.set(ControlMode.PercentOutput, triggerSpeed); 
  }

  public void triggerStop() {
    trigger.set(ControlMode.PercentOutput, 0);
  } 

  public void triggerReverse() {
    trigger.set(ControlMode.PercentOutput, -.4); 
  }

  private int readyCount = 0;
  public boolean flyWheelReady() {
    boolean atSpeed = Math.abs(flyWheel.getClosedLoopError()) < 500;
    if (atSpeed)
      readyCount += 1;
    else
      readyCount = 0;

    return (readyCount > 10);
  } 

  public void indexReverse(){ 
      trigger.set(ControlMode.PercentOutput, -.5); 
      flyWheel.set(ControlMode.PercentOutput, -.15);
  } 

  public void indexStop(){ 
    triggerStop();
    flywheelStop();
  }

  private void setLowIndex(int value) {
    if (value > 3) {
      value = 3;
    } else if (value < -3) {
      value = -3;
    }

    speedLowIndex = value;
    SmartDashboard.putNumber("Shooter/Speed/Low", getLowSpeed());
  }

  public void increaseHighIndex() {
    setHighIndex(speedHighIndex + 1);
  }

  public void decreaseHighIndex() {
    setHighIndex(speedHighIndex - 1);
  }

  private void setHighIndex(int value) {
    if (value > 3) {
      value = 3;
    } else if (value < -3) {
      value = -3;
    }

    speedHighIndex = value;
    SmartDashboard.putNumber("Shooter/Speed/High", getHighSpeed());
  }

  public boolean isStopped() {
    return flyWheel.getControlMode() == ControlMode.PercentOutput;
  }

  private void flywheelSpeed(int speed) {
    readyCount = 0;
    if (speed == 0) {
      flyWheel.set(ControlMode.PercentOutput, 0);
    } else {
      flyWheel.set(ControlMode.Velocity, speed);
    }
  }
}
