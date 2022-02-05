// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.sensor.UltrasonicSensor;

public class UltrasonicRead extends CommandBase {
  private UltrasonicSensor ultraSonicSensor;

  public UltrasonicRead() {
    ultraSonicSensor = new UltrasonicSensor(Constants.Sensors.ULTRASONICSENSOR);
  }

  @Override
  public void execute() {
    ShuffleboardTab tab = Shuffleboard.getTab("Sensors");
    tab.add("Ultrasonic Sensor", ultraSonicSensor.GetValue());
  }
}
