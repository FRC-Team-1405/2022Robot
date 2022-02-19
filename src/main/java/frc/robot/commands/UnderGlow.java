package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.sensor.LEDStrip;

public class UnderGlow extends CommandBase {
  private LEDStrip ledStrip;
  private Color teamColor;

  public UnderGlow(LEDStrip ledStrip) {
    this.ledStrip = ledStrip;
  }

  @Override
  public void execute() {
    Alliance team = DriverStation.getAlliance();

    AddressableLEDBuffer addressableLEDBuffer = ledStrip.getLedBuffer();

    if(team == Alliance.Blue) {
      teamColor = ledStrip.setPercentBrightness(Color.kBlue, Constants.UnderGlow.BRIGHTNESS);
    }
    else if(team == Alliance.Red) {
      teamColor = ledStrip.setPercentBrightness(Color.kRed, Constants.UnderGlow.BRIGHTNESS);
    }
    else {
      teamColor = ledStrip.setPercentBrightness(Color.kGreen, Constants.UnderGlow.BRIGHTNESS);
    }

    for(int i = 0; i < Constants.UnderGlow.LEDCOUNT; i++) {
      addressableLEDBuffer.setLED(i + Constants.UnderGlow.LEDSTART, teamColor);
    }

    ledStrip.displayLEDBuffer(addressableLEDBuffer);
  }

  @Override
  public boolean runsWhenDisabled(){
    return true;
  }
}