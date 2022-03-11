// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class SwerveDrive extends SubsystemBase implements SwerveSubsystem {
  //I think we can use these values as our speedlimit, if we make them configureable on Shuffleboard 
  public double maxVelocity; 
  public double maxAngularSpeed; 
   
  //Our swerve modules 
  private final SwerveModule frontLeft = new SwerveModule(Constants.SwerveBase.DRIVEFRONTLEFT, Constants.SwerveBase.ROTATIONFRONTLEFT, Constants.SwerveBase.ENCODERFRONTLEFT, 45); 
  private final SwerveModule frontRight = new SwerveModule(Constants.SwerveBase.DRIVEFRONTRIGHT, Constants.SwerveBase.ROTATIONFRONTRIGHT, Constants.SwerveBase.ENCODERFRONTRIGHT, -45); 
  private final SwerveModule backLeft = new SwerveModule(Constants.SwerveBase.DRIVEBACKLEFT, Constants.SwerveBase.ROTATIONBACKLEFT, Constants.SwerveBase.ENCODERBACKLEFT, -45); 
  private final SwerveModule backRight = new SwerveModule(Constants.SwerveBase.DRIVEBACKRIGHT, Constants.SwerveBase.ROTATIONBACKRIGHT, Constants.SwerveBase.ENCODERBACKRIGHT, 45); 
  //Our gyro (used to determine robot heading)
  private final AHRS gyro = new AHRS(SPI.Port.kMXP); 

  private final SwerveDriveOdometry odometry = 
          new SwerveDriveOdometry(Constants.SwerveBase.KINEMATICS, gyro.getRotation2d()); 

  public SwerveDrive() {
    //I am making the maxVelocity configurable so we can ajdust our "speedlimit"
    Preferences.initDouble("SwerveDrive/Speed Limit", 6); 
    maxVelocity = Preferences.getDouble("SwerveDrive/Speed Limit", 6) ;
    Preferences.initDouble("SwerveDrive/Rotation Speed Limit", 6.5); 
    maxAngularSpeed = Preferences.getDouble("SwerveDrive/Rotation Speed Limit", 6.5) ;
    //It may be useful to reset the gyro like this every boot-up. I believe we did this our old code
    gyro.reset();

    enableFieldOriented(isFieldOrientedEnabled);
  }


  @Override
  public void periodic() {
    updateOdometry();
  }

  public void drive(double xPercent, double yPercent, double rotationPercent){ 
    driveSpeed(xPercent * maxVelocity, yPercent * maxVelocity, rotationPercent * maxAngularSpeed, fieldOriented());
  } 

  public void driveSpeed(double xSpeed, double ySpeed, double rotationSpeed, boolean fieldOriented){
    // SmartDashboard.putNumber("DriveTo/Speed/x", xSpeed);
    // SmartDashboard.putNumber("DriveTo/Speed/y", ySpeed);
    // SmartDashboard.putNumber("DriveTo/Speed/z", rotationSpeed);

    SwerveModuleState[] swerveModuleStates = Constants.SwerveBase.KINEMATICS.toSwerveModuleStates(
      fieldOriented
      ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, 
                                              ySpeed, 
                                              rotationSpeed, 
                                              Rotation2d.fromDegrees(gyro.getAngle())) 
      : new ChassisSpeeds(xSpeed, 
                          ySpeed, 
                          rotationSpeed)); 
    //This function should limit our speed to the value we set (maxVelocity)

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxVelocity); 

    // SmartDashboard.putNumber("angle", gyro.getAngle()); 
    
    setModuleStates(swerveModuleStates);
  }

  public void updateOdometry(){ 
    odometry.update( Rotation2d.fromDegrees(-gyro.getAngle()), 
                     frontLeft.getState(), 
                     frontRight.getState(), 
                     backLeft.getState(), 
                     backRight.getState());
    // SmartDashboard.putNumber("SwerveDrive/Pose/X", Units.metersToInches(odometry.getPoseMeters().getX()));
    // SmartDashboard.putNumber("SwerveDrive/Pose/Y", Units.metersToInches(odometry.getPoseMeters().getY()));
    // SmartDashboard.putNumber("SwerveDrive/Pose/Z", odometry.getPoseMeters().getRotation().getDegrees());
  }

  public boolean fieldOriented(){ 
    return (gyro != null && isFieldOrientedEnabled) ? true : false;
  }

  protected boolean isFieldOrientedEnabled = true;
  public void enableFieldOriented(boolean value){
    isFieldOrientedEnabled = value;
    SmartDashboard.putBoolean("Drive by Field Oriented", isFieldOrientedEnabled);
  }
  // FieldOriented and Gyro control mapped to control stick button on a true/false boolean

  public void setStartLocation(Pose2d pose) {
    gyro.setAngleAdjustment(pose.getRotation().getDegrees() - gyro.getAngle());
    odometry.resetPosition( pose, pose.getRotation() );
  } 

  public Pose2d getPose(){ 
    Pose2d pose = odometry.getPoseMeters();
    return new Pose2d( pose.getX(), pose.getY(), Rotation2d.fromDegrees(gyro.getAngle()) ); 
  } 

  public void setModuleStates(SwerveModuleState[] states){ 
    frontLeft.setDesiredState(states[0]);
    frontRight.setDesiredState(states[1]); 
    backLeft.setDesiredState(states[2]);
    backRight.setDesiredState(states[3]);
  }

  @Override
  public void setPose(Pose2d pose) {
    odometry.resetPosition( pose, pose.getRotation() );
  }

  public SwerveDriveKinematics getKinematics() {
    return Constants.SwerveBase.KINEMATICS ;
  }

  public double getMaxSpeed() {
    return 3.0;
}

public double getMaxAcceleration() {
    return 1.0;
}

public double getMaxAngularSpeed() {
    return Math.PI*2;
}

public double getMaxAngularAcceleration() {
    return Math.PI;
} 
}
