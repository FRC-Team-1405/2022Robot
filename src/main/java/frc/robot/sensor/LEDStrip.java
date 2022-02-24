package frc.robot.sensor;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LEDStrip {
    private final AddressableLED led;
    private AddressableLEDBuffer ledBuffer;

    public LEDStrip(int port, int ledCount) {
        led = new AddressableLED(port);
        ledBuffer = new AddressableLEDBuffer(ledCount);
        led.setLength(ledCount);
        led.setData(ledBuffer);
        //led.start continuously sends to the leds
        led.start(); 
    }
    
    public AddressableLEDBuffer getLedBuffer() {
        return ledBuffer;
    }

    public void setLedBuffer(AddressableLEDBuffer buffer) {
        ledBuffer = buffer;
    }

    public void displayLEDBuffer() {
        led.setData(ledBuffer);
    }
}