// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.trajectories.GenerateTrajectory;

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
        public final static double MINVOLTAGE = 9;
        //LEDCOUNT has to be a multiple of 3
        public final static int METERCOUNT = 15;
        public final static int LEDCOUNT = METERCOUNT * 2;

        public final static double BRIGHTNESS = 0.2;
    };

    public class UnderGlow{
        //This comment is old so it might not be right
        //The under glow has 43 leds
        public final static int LEDCOUNT = 0;

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
        public final static int DRIVEFRONTLEFT = 1;
        public final static int DRIVEFRONTRIGHT = 2;
        public final static int DRIVEBACKLEFT = 3;
        public final static int DRIVEBACKRIGHT = 4; 

        public final static int ROTATIONFRONTLEFT = 21;
        public final static int ROTATIONFRONTRIGHT = 22;
        public final static int ROTATIONBACKLEFT = 23;
        public final static int ROTATIONBACKRIGHT = 24; 

        public final static int ENCODERFRONTLEFT = 31;
        public final static int ENCODERFRONTRIGHT = 32;
        public final static int ENCODERBACKLEFT = 33;
        public final static int ENCODERBACKRIGHT = 34; 

        public final static double WHEELRADIUS = 0.0508; 
        public final static double WHEELCIRCUMFERENCE = WHEELRADIUS * 2 * Math.PI;
        public final static double DRIVEMOTORENCODERRESOLUTION = 2048;  
        //For converting 100 milleseconds (heretofore referred to as 'ms') to seconds 
        public static final double TIMECONSTANTFORCONVERSION = 10; 
        public static final double GEARATIO = 6;

        //A simple conversion formula to turn encoder velocity (sensor units/100ms) to meters per second 
        public static final double VELOCITYMETERS = 1 / DRIVEMOTORENCODERRESOLUTION * WHEELCIRCUMFERENCE * 1 / GEARATIO * TIMECONSTANTFORCONVERSION;
        // A simple conversion formula to turn meters per second to encoder velocity
        public static final double VELOCITYSENSOR = DRIVEMOTORENCODERRESOLUTION * 1 / WHEELCIRCUMFERENCE * GEARATIO * 1 / TIMECONSTANTFORCONVERSION;
    
        public static final double MAXANGULARSPEED = 1; 
        public static final double MAXANGULARACCELERARTION = 1; 
    
        public static final SwerveDriveKinematics KINEMATICS = 
                new SwerveDriveKinematics(
                    new Translation2d(Units.inchesToMeters(13),  Units.inchesToMeters(-13)),    // Front Left
                    new Translation2d(Units.inchesToMeters(13),  Units.inchesToMeters(13)),     //Front Right
                    new Translation2d(Units.inchesToMeters(-13), Units.inchesToMeters(-13)),    //Back Left
                    new Translation2d(Units.inchesToMeters(-13), Units.inchesToMeters(13)));    //Back Right
     } 

     public final class Shooter{ 
        public final static double INDEX_SPEED = .85; 
    }

    // X+ => forward    Y+ => left  (0,0) => Hub center CCW => +
    // Cago angles are towards the goal
    public final static class FieldPosition {
        public static Pose2d Target          = new Pose2d( Units.inchesToMeters(   0), Units.inchesToMeters(   0), Rotation2d.fromDegrees(0) );
        public static Pose2d Cargo_Left      = GenerateTrajectory.pose2dToOrigin( Units.inchesToMeters(-129), Units.inchesToMeters(  82) );
        public static Pose2d Cargo_Center    = GenerateTrajectory.pose2dToOrigin( Units.inchesToMeters(-125), Units.inchesToMeters(  88) );
        public static Pose2d Cargo_Right     = GenerateTrajectory.pose2dToOrigin( Units.inchesToMeters( -26), Units.inchesToMeters(-151) );
        public static Pose2d Cargo_Back      = GenerateTrajectory.pose2dToOrigin( Units.inchesToMeters(-282), Units.inchesToMeters(-118) );
        public static Pose2d Tarmac_LeftLeft     = GenerateTrajectory.pose2dToOrigin( Units.inchesToMeters(-60), Units.inchesToMeters( 68) );
        public static Pose2d Tarmac_LeftCenter   = GenerateTrajectory.pose2dToOrigin( Units.inchesToMeters(-89), Units.inchesToMeters( 34) );
        public static Pose2d Tarmac_LeftRight    = GenerateTrajectory.pose2dToOrigin( Units.inchesToMeters(-90), Units.inchesToMeters(-11) );
        public static Pose2d Tarmac_RightLeft    = GenerateTrajectory.pose2dToOrigin( Units.inchesToMeters(-68), Units.inchesToMeters(-60) );
        public static Pose2d Tarmac_RightCenter  = GenerateTrajectory.pose2dToOrigin( Units.inchesToMeters(-32), Units.inchesToMeters(-91) );
        public static Pose2d Tarmac_RightRight   = GenerateTrajectory.pose2dToOrigin( Units.inchesToMeters( 10), Units.inchesToMeters(-92) );
        public static Pose2d Waypoint_Inside     = GenerateTrajectory.pose2dToOrigin( Units.inchesToMeters(-80), Units.inchesToMeters(  0) );
   }
}
