// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class SwerveDrive extends SubsystemBase {
  /** Creates a new SwerveDrive. */ 
  //I think we can use these values as our speedlimit, if we make them configureable on Shuffleboard 
  public static double maxVelocity; 
  public static double maxAngularSpeed; 
  
  /** These variables store the location of each swerve module relative to the center of the robot. 
  Currently, I am just copying the ones from the example code. */
  private final Translation2d frontLeftLocation = new Translation2d(Units.inchesToMeters(13), Units.inchesToMeters(-13)); 
  private final Translation2d frontRightLocation = new Translation2d(Units.inchesToMeters(13), Units.inchesToMeters(13)); 
  private final Translation2d backLeftLocation = new Translation2d(Units.inchesToMeters(-13), Units.inchesToMeters(-13)); 
  private final Translation2d backRightLocation = new Translation2d(Units.inchesToMeters(-13), Units.inchesToMeters(13)); 
  //Our swerve modules 
  private final SwerveModule frontLeft = new SwerveModule(Constants.SwerveBase.driveFrontLeft, Constants.SwerveBase.azimuthFrontLeft, Constants.SwerveBase.encoderFrontLeft, 45); 
  private final SwerveModule frontRight = new SwerveModule(Constants.SwerveBase.driveFrontRight, Constants.SwerveBase.azimuthFrontRight, Constants.SwerveBase.encoderFrontRight, -45); 
  private final SwerveModule backLeft = new SwerveModule(Constants.SwerveBase.driveBackLeft, Constants.SwerveBase.azimuthBackLeft, Constants.SwerveBase.encoderBackLeft, -45); 
  private final SwerveModule backRight = new SwerveModule(Constants.SwerveBase.driveBackRight, Constants.SwerveBase.azimuthBackRight, Constants.SwerveBase.encoderBackRight, 45); 
  //Our gyro (used to determine robot heading)
  private final AHRS gyro = new AHRS(SPI.Port.kMXP); 
  
  private final SwerveDriveKinematics kinematics = 
          new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, 
          backLeftLocation, backRightLocation);

  private final SwerveDriveOdometry odometry = 
          new SwerveDriveOdometry(kinematics, gyro.getRotation2d()); 



  public SwerveDrive() {
    //I am making the maxVelocity configurable so we can ajdust our "speedlimit"
    Preferences.initDouble("SwerveDrive/Speed Limit", 3); 
    maxVelocity = Preferences.getDouble("SwerveDrive/Speed Limit", 3) ;
    Preferences.initDouble("SwerveDrive/Rotation Speed Limit", 2); 
    maxAngularSpeed = Preferences.getDouble("SwerveDrive/Rotation Speed Limit", 2) ;
    //It may be useful to reset the gyro like this every boot-up. I believe we did this our old code
    gyro.reset();

    enableFieldOriented(isFieldOrientedEnabled);
  }

  public void drive(double xSpeed, double ySpeed, double rotationSpeed){ 
    SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(
      fieldOriented() 
      ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed * maxVelocity, 
                                              ySpeed * maxVelocity, 
                                              rotationSpeed * maxAngularSpeed, 
                                              Rotation2d.fromDegrees(gyro.getAngle())) 
      : new ChassisSpeeds(xSpeed * maxVelocity, 
                          ySpeed * maxVelocity, 
                          rotationSpeed * maxAngularSpeed)); 
    //This function should limit our speed to the value we set (maxVelocity)

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxVelocity); 

    SmartDashboard.putNumber("angle", gyro.getAngle()); 
    
    setModuleStates(swerveModuleStates);
  } 

public void updateOdometry(){ 
  odometry.update(gyro.getRotation2d(), frontLeft.getState(), 
                                        frontRight.getState(), 
                                        backLeft.getState(), 
                                        backRight.getState());
  }

  public boolean fieldOriented(){ 
    return (gyro != null && isFieldOrientedEnabled)  ? true : false;
  }

  protected boolean isFieldOrientedEnabled = true;
  public void enableFieldOriented(boolean value){
    isFieldOrientedEnabled = value;
    SmartDashboard.putBoolean("Drive by Field Oriented", isFieldOrientedEnabled);
  }
  // FieldOriented and Gyro control mapped to control stick button on a true/false boolean

  public void setStartLocation(double yPos, double xPos, double rotation) {
    gyro.setAngleAdjustment(rotation);
  } 

  public Pose2d getPose(){ 
    return odometry.getPoseMeters(); 
  } 
  
  public SwerveDriveKinematics getKinematics(){ 
    return kinematics; 
  }

  public void setModuleStates(SwerveModuleState[] states){ 
    frontLeft.setDesiredState(states[0]);
    frontRight.setDesiredState(states[1]); 
    backLeft.setDesiredState(states[2]);
    backRight.setDesiredState(states[3]);
  }
}
