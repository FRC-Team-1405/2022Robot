package frc.robot.sensor;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;

public class UltrasonicSensor {
    private final AnalogInput ultrasonicSensor;

    public UltrasonicSensor(int port) {
        ultrasonicSensor = new AnalogInput(port);
    }

    public double GetValue() {
        //voltageScaleFactor allows us to compensate for differences in supply voltage.
        double voltageScaleFactor = 5 / RobotController.getVoltage5V();
        double rawValue = ultrasonicSensor.getValue();

        //This is from https://www.maxbotix.com/firstrobotics
        return rawValue * voltageScaleFactor * 0.125 / 100;
    }
}
