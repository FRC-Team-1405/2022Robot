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

    //Set the offset for writing to the AddressableLEDBuffer with the ledCountOffset
    int ledCountOffset = 0;
    for (LED ledCommand : ledCommands) {
      ledCommand.initialize(ledCountOffset);
      ledCountOffset += ledCommand.numLEDs;
    }

    ledStrip = new LEDStrip(port, ledCountOffset);
  }

  @Override
  public void execute() {
    //Loops over all of the led Programs
    for (LED ledCommand : ledCommands) {
      ledStrip.setLedBuffer(ledCommand.writeData(ledStrip.getLedBuffer())); 
    }
    ledStrip.displayLEDBuffer();
  }

  //Without this the command wouldn’t run when scheduled in Robot.java robotInit
  @Override
  public boolean runsWhenDisabled(){
    return true;
  }
}
