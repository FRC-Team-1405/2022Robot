// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lib.LEDs.LED;
import frc.robot.sensor.LEDStrip;

public class LEDManager extends CommandBase {
  private LED[] ledCommands;
  private LEDStrip ledStrip;

  public LEDManager(int port, LED[] ledCommands) {
    this.ledCommands = ledCommands;

    //Set the offset for writing to the AddressableLEDBuffer
    int ledCount = 0;
    for (LED ledCommand : ledCommands) {
      ledCommand.initialize(ledCount);
      ledCount += ledCommand.numLEDs;
    }

    ledStrip = new LEDStrip(port, ledCount);
  }

  @Override
  public void execute() {
    for (LED ledCommand : ledCommands) {
      ledStrip.setLedBuffer(ledCommand.writeData(ledStrip.getLedBuffer())); 
    }
    ledStrip.displayLEDBuffer();
  }

  @Override
  public boolean runsWhenDisabled(){
    return true;
  }
}
