// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public final class CANID {
        public final static int FLYWHEEL =  10; 
        public final static int TRIGGER = 11;
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
        //LEDCOUNT has to be a multiple of 3
        public final static int METERCOUNT = 15;
        public final static int LEDCOUNT = METERCOUNT * 2;

        public final static double BRIGHTNESS = 0.2;
    };

    public class UnderGlow{
        //The under glow has 43 leds
        public final static int LEDCOUNT = 10;

        public final static double BRIGHTNESS = 0.2;
    }
    
    public final class PWMPort{
        public final static int LEDPORT = 8;
        public final static int TOTALLEDCOUNT = BatteryMonitor.LEDCOUNT + UnderGlow.LEDCOUNT;
    };
    public final class Sensors{
        public final static int ULTRASONICSENSOR = 3;
    }
    public final static class SwerveBase {
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

        public final static double wheelRadius = 0.0508; 
        public final static double wheelCircumference = wheelRadius * 2 * Math.PI;
        public final static double driveMotorEncoderResolution = 2048;  
        //For converting 100 milleseconds (heretofore referred to as 'ms') to seconds 
        public static final double timeConstantForConversion = 10; 
        public static final double gearatio = 6;

        //A simple conversion formula to turn encoder velocity (sensor units/100ms) to meters per second 
        public static final double velocityMeters = 1/driveMotorEncoderResolution * wheelCircumference * 1/gearatio * timeConstantForConversion;
        // A simple conversion formula to turn meters per second to encoder velocity
        public static final double velocitySensor = driveMotorEncoderResolution * 1/wheelCircumference * gearatio * 1/timeConstantForConversion;
    
        public static final double maxAngularSpeed = 1; 
        public static final double maxAngularAccelerartion = 1; 
    
        public static final SwerveDriveKinematics kinematics = 
                new SwerveDriveKinematics(
                    new Translation2d(Units.inchesToMeters(13),  Units.inchesToMeters(-13)),    // Front Left
                    new Translation2d(Units.inchesToMeters(13),  Units.inchesToMeters(13)),     //Front Right
                    new Translation2d(Units.inchesToMeters(-13), Units.inchesToMeters(-13)),    //Back Left
                    new Translation2d(Units.inchesToMeters(-13), Units.inchesToMeters(13)));    //Back Right
      
    
     } 

     public final class Shooter{ 
        public final static double INDEX_SPEED = .75; 
    }
}
