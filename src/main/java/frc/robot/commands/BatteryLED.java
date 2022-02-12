// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.lib.MathTools;
import frc.robot.sensor.LEDStrip;

public class BatteryLED extends CommandBase {
  private LEDStrip ledStrip;
  private AddressableLEDBuffer addressableLEDBuffer;
  
  public BatteryLED(LEDStrip ledStrip) {
    this.ledStrip = ledStrip;
    this.addressableLEDBuffer = ledStrip.getLedBuffer() ;
  }

  @Override
  public void execute() {
    double voltage = RobotController.getBatteryVoltage();
    
    int numberOfLeds = (int)MathTools.map(voltage, Constants.BatteryMonitor.MINVOLTAGE, Constants.BatteryMonitor.MAXVOLTAGE, 0, Constants.BatteryMonitor.LEDSTART);

    int red = Constants.BatteryMonitor.LEDCOUNT / 3;
    int yellow = red + red;

    for (var i = 0; i < Constants.BatteryMonitor.LEDCOUNT; i++) {
      if(i < red) {
        addressableLEDBuffer.setLED(i + Constants.BatteryMonitor.LEDSTART, (i < numberOfLeds ? ledStrip.devideColor(Color.kRed) : ledStrip.devideColor(Color.kBlack)));
      }
      else if(i < yellow) {
        addressableLEDBuffer.setLED(i + Constants.BatteryMonitor.LEDSTART, (i < numberOfLeds ? ledStrip.devideColor(Color.kYellow) : ledStrip.devideColor(Color.kBlack)));
      }
      else {
        addressableLEDBuffer.setLED(i + Constants.BatteryMonitor.LEDSTART, (i < numberOfLeds ? ledStrip.devideColor(Color.kGreen) : ledStrip.devideColor(Color.kBlack)));
      }
    }

    ledStrip.displayLEDBuffer(addressableLEDBuffer);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
