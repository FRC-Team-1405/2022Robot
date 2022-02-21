// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.LEDs;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;

public class UnderGlow extends LED {
    private int offset;

    public UnderGlow(int numLEDs) {
      super(numLEDs);
    }

    @Override
    public void initialize(int offset) {
        this.offset = offset;
    }

    @Override
    public AddressableLEDBuffer writeData(AddressableLEDBuffer buffer) { 
        Alliance team = DriverStation.getAlliance();

        Color teamColor;
        if(team == Alliance.Blue) {
          teamColor = super.setPercentBrightness(Color.kBlue, Constants.UnderGlow.BRIGHTNESS);
        } else if(team == Alliance.Red) {
          teamColor = super.setPercentBrightness(Color.kRed, Constants.UnderGlow.BRIGHTNESS);
        } else {
          teamColor = super.setPercentBrightness(Color.kGreen, Constants.UnderGlow.BRIGHTNESS);
        }
    
        for(int i = 0; i < Constants.UnderGlow.LEDCOUNT; i++) {
          buffer.setLED(i + offset, teamColor);
        }
    
        return buffer;
    }
}
