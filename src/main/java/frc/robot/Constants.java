// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {
    public final class CAN_ID {
        public final static int FLYWHEEL =  10; 
        public final static int PICKUP = 5; 
        public final static int INTAKE_DROPPER = 12;
    };

    public final class Controller {
        public final static int DRIVER      = 0;
        public final static int OPERATOR    = 1;
        public final static int TESTER      = 2;
    };

    public final class BatteryMonitor {
        public final static double MAXVOLTAGE = 12.5;
        public final static double MINVOLTAGE = 7;
        //ledCount has to be a multiple of 3
        public final static int LEDSTART = 0;
        public final static int LEDSTOP = 15;
        public final static int LEDCOUNT = LEDSTOP - LEDSTART;
        public final static int BRIGHTNESS = 100;
    };

    public final class PWM_Port{
        public final static int LEDS = 8;
        public final static int TOTALLEDCOUNT = BatteryMonitor.LEDCOUNT;
    };
    public final class SwerveBase {
        public final static int DRIVEFRONTLEFT = 1;
        public final static int DRIVEFRONTRIGHT = 2;
        public final static int DRIVEBACKLEFT = 3;
        public final static int DRIVEBACKRIGHT = 4; 

        public final static int AZIMUTHFRONTLEFT = 21;
        public final static int AZIMUTHFRONTRIGHT = 22;
        public final static int AZIMUTHBACKLEFT = 23;
        public final static int AZIMUTHBACKRIGHT = 24; 

        public final static int ENCODERFRONTLEFT = 31;
        public final static int ENCODERFRONTRIGHT = 32;
        public final static int ENCODERBACKLEFT = 33;
        public final static int ENCODERBACKRIGHT = 34;
    }

    public final class Sensors{
        public final static int ULTRASONICSENSOR = 3;
    }
}
