// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.LEDs;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

/*
This is an interface class for creating programs that work with the LEDManager
*/
public class LED {
    public final int numLEDs;

    //This is to keep track of the led offset in the battery monitor
    LED(int numLEDs) {
        this.numLEDs = numLEDs;
    }

    //Sets the offset to apply to buffer set calls
    public void initialize(int offset) {}

    //Basically like an execute call
    public AddressableLEDBuffer writeData(AddressableLEDBuffer buffer) { return buffer; }

    //Used to set the brightness of every color
    public static Color setPercentBrightness(Color color, double brightness) {
        double blue = color.blue * brightness;
        double green = color.green * brightness;
        double red = color.red * brightness;
    
        return new Color(red, green, blue);
    }
}
