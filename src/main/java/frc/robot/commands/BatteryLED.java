// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.lib.MathTools;
import frc.robot.sensor.LEDStrip;

public class BatteryLED extends CommandBase {
  private LEDStrip ledStrip;
  
  public BatteryLED(LEDStrip ledStrip) {
    this.ledStrip = ledStrip;
    // SmartDashboard.putNumber("LedVoltageTest", 0);
    setName("BatteryLED");
  }

  @Override
  public void execute() {
    double voltage = RobotController.getBatteryVoltage();
    // double voltage = SmartDashboard.getNumber("LedVoltageTest", 0);

    voltage = voltage < Constants.BatteryMonitor.MINVOLTAGE ? Constants.BatteryMonitor.MINVOLTAGE : voltage;

    //Make sure addressableLEDBuffer is not overwriting the current buffer
    AddressableLEDBuffer addressableLEDBuffer = ledStrip.getLedBuffer();

    int numberOfLeds = (int)MathTools.map(voltage, Constants.BatteryMonitor.MINVOLTAGE, Constants.BatteryMonitor.MAXVOLTAGE, 1, Constants.BatteryMonitor.METERCOUNT);

    // Red
    for (int i = 0; i < Constants.BatteryMonitor.SEGMENTLENGTH; i++) {
      addressableLEDBuffer.setLED(i + Constants.BatteryMonitor.LEDSTART + Constants.BatteryMonitor.REDSTART, (i + Constants.BatteryMonitor.REDSTART < numberOfLeds ? ledStrip.setPercentBrightness(Color.kRed, Constants.BatteryMonitor.BRIGHTNESS) : ledStrip.setPercentBrightness(Color.kBlack, Constants.BatteryMonitor.BRIGHTNESS)));
      addressableLEDBuffer.setLED(Constants.BatteryMonitor.LEDCOUNT - (i + Constants.BatteryMonitor.LEDSTART + Constants.BatteryMonitor.REDSTART + 1), (i + Constants.BatteryMonitor.REDSTART < numberOfLeds ? ledStrip.setPercentBrightness(Color.kRed, Constants.BatteryMonitor.BRIGHTNESS) : ledStrip.setPercentBrightness(Color.kBlack, Constants.BatteryMonitor.BRIGHTNESS)));
    }

    // Yellow
    for (int i = 0; i < Constants.BatteryMonitor.SEGMENTLENGTH; i++) {
      addressableLEDBuffer.setLED(i + Constants.BatteryMonitor.LEDSTART + Constants.BatteryMonitor.YELLOWSTART, (i + Constants.BatteryMonitor.YELLOWSTART < numberOfLeds ? ledStrip.setPercentBrightness(Color.kYellow, Constants.BatteryMonitor.BRIGHTNESS) : ledStrip.setPercentBrightness(Color.kBlack, Constants.BatteryMonitor.BRIGHTNESS)));
      addressableLEDBuffer.setLED(Constants.BatteryMonitor.LEDCOUNT - (i + Constants.BatteryMonitor.LEDSTART + Constants.BatteryMonitor.YELLOWSTART + 1), (i + Constants.BatteryMonitor.YELLOWSTART < numberOfLeds ? ledStrip.setPercentBrightness(Color.kYellow, Constants.BatteryMonitor.BRIGHTNESS) : ledStrip.setPercentBrightness(Color.kBlack, Constants.BatteryMonitor.BRIGHTNESS)));
    }

    // Green
    for (int i = 0; i < Constants.BatteryMonitor.SEGMENTLENGTH; i++) {
      addressableLEDBuffer.setLED(i + Constants.BatteryMonitor.LEDSTART + Constants.BatteryMonitor.GREENSTART, (i + Constants.BatteryMonitor.GREENSTART < numberOfLeds ? ledStrip.setPercentBrightness(Color.kGreen, Constants.BatteryMonitor.BRIGHTNESS) : ledStrip.setPercentBrightness(Color.kBlack, Constants.BatteryMonitor.BRIGHTNESS)));
      addressableLEDBuffer.setLED(Constants.BatteryMonitor.LEDCOUNT - (i + Constants.BatteryMonitor.LEDSTART + Constants.BatteryMonitor.GREENSTART + 1), (i + Constants.BatteryMonitor.GREENSTART < numberOfLeds ? ledStrip.setPercentBrightness(Color.kGreen, Constants.BatteryMonitor.BRIGHTNESS) : ledStrip.setPercentBrightness(Color.kBlack, Constants.BatteryMonitor.BRIGHTNESS)));
    }

    ledStrip.displayLEDBuffer(addressableLEDBuffer);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
