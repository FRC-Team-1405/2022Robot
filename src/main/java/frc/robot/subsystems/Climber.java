// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private WPI_TalonSRX motorLeft = new WPI_TalonSRX(Constants.CANID.CLIMBER_LEFT);
  private WPI_TalonSRX motorRight = new WPI_TalonSRX(Constants.CANID.CLIMBER_RIGHT);

  private static double maxSpeed = 1.0;
  private boolean climbEnabled = false;

  public Climber() {
    Preferences.initDouble("Climber/MaxSpeed", maxSpeed);
    maxSpeed = Preferences.getDouble("Climber/MaxSpeed", maxSpeed);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void enableClimber(){
    climbEnabled = true;
  }

  public void disableClimber(){
    climbEnabled = false;
    motorLeft.set(ControlMode.PercentOutput, 0.0);
    motorRight.set(ControlMode.PercentOutput, 0.0);
  }

  public boolean isClimbEnabled(){
    return climbEnabled;
  }
  
  public void left(double speed){
    if (climbEnabled) {
      motorLeft.set(ControlMode.PercentOutput, (maxSpeed * speed));
    }
  }

  public void right(double speed){
    if (climbEnabled) {
      motorRight.set(ControlMode.PercentOutput, (maxSpeed * speed));
    }
  }
  
}

