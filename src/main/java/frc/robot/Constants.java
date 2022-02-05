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
        public final static int driveFrontLeft = 1;
        public final static int driveFrontRight = 2;
        public final static int driveBackLeft = 3;
        public final static int driveBackRight = 4; 

        public final static int azimuthFrontLeft = 21;
        public final static int azimuthFrontRight = 22;
        public final static int azimuthBackLeft = 23;
        public final static int azimuthBackRight = 24; 

        public final static int encoderFrontLeft = 31;
        public final static int encoderFrontRight = 32;
        public final static int encoderBackLeft = 33;
        public final static int encoderBackRight = 34;
    }
}
