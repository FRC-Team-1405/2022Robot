// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.LEDs;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import frc.robot.lib.MathTools;

public class BatteryLED extends LED {
  private int numLEDs;
  private int segmentLength;
  private int offset;

  private int redOffset, yellowOffset, greenOffset;

  //numLEDs has to be a multiple of 3
  public BatteryLED(int numLEDs) {
    super(numLEDs);
    this.numLEDs = numLEDs;

    //Divided by 2 because we have 2 battery monitor strips
    segmentLength = numLEDs / 2 / 3;

    redOffset = 0;
    yellowOffset = segmentLength;
    greenOffset = segmentLength * 2;

    // SmartDashboard.putNumber("LedVoltageTest", 0);
  }

  @Override
  public void initialize(int offset) {
      this.offset = offset;
  }

  @Override
  public AddressableLEDBuffer writeData(AddressableLEDBuffer buffer) {
    double voltage = RobotController.getBatteryVoltage();
    // double voltage = SmartDashboard.getNumber("LedVoltageTest", 0);

    voltage = voltage < Constants.BatteryMonitor.MINVOLTAGE ? Constants.BatteryMonitor.MINVOLTAGE : voltage;

    int numberOfLeds = (int)MathTools.map(voltage, Constants.BatteryMonitor.MINVOLTAGE, Constants.BatteryMonitor.MAXVOLTAGE, 1, numLEDs / 2);

    // Red
    for (int i = 0; i < segmentLength; i++) {
      buffer.setLED(i + offset + redOffset, (i + redOffset < numberOfLeds ? super.setPercentBrightness(Color.kRed, Constants.BatteryMonitor.BRIGHTNESS) : super.setPercentBrightness(Color.kBlack, Constants.BatteryMonitor.BRIGHTNESS)));
      buffer.setLED(numLEDs - (i + offset + redOffset + 1), (i + redOffset < numberOfLeds ? super.setPercentBrightness(Color.kRed, Constants.BatteryMonitor.BRIGHTNESS) : super.setPercentBrightness(Color.kBlack, Constants.BatteryMonitor.BRIGHTNESS)));
    }

    // Yellow
    for (int i = 0; i < segmentLength; i++) {
      buffer.setLED(i + offset + yellowOffset, (i + yellowOffset < numberOfLeds ? super.setPercentBrightness(Color.kYellow, Constants.BatteryMonitor.BRIGHTNESS) : super.setPercentBrightness(Color.kBlack, Constants.BatteryMonitor.BRIGHTNESS)));
      buffer.setLED(numLEDs - (i + offset + yellowOffset + 1), (i + yellowOffset < numberOfLeds ? super.setPercentBrightness(Color.kYellow, Constants.BatteryMonitor.BRIGHTNESS) : super.setPercentBrightness(Color.kBlack, Constants.BatteryMonitor.BRIGHTNESS)));
    }

    // Green
    for (int i = 0; i < segmentLength; i++) {
      buffer.setLED(i + offset + greenOffset, (i + greenOffset < numberOfLeds ? super.setPercentBrightness(Color.kGreen, Constants.BatteryMonitor.BRIGHTNESS) : super.setPercentBrightness(Color.kBlack, Constants.BatteryMonitor.BRIGHTNESS)));
      buffer.setLED(numLEDs - (i + offset + greenOffset + 1), (i + greenOffset < numberOfLeds ? super.setPercentBrightness(Color.kGreen, Constants.BatteryMonitor.BRIGHTNESS) : super.setPercentBrightness(Color.kBlack, Constants.BatteryMonitor.BRIGHTNESS)));
    }

    return buffer;
  }
}
