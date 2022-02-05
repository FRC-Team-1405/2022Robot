// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

//CTRE deps
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
//WPILIB deps
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveModule extends SubsystemBase {
  /** A swerve module must have a drive motor and a steering motor. The drive motor gives the power to 
  the wheel of the swerve module, and the steering motor points the wheel in the direction it should 
  go. It can be thought of as a vector, with the steering motor controlling the direction and the 
  drive motor controlling the magnitude (oh yeah!). */ 

  //Our drive motor
  private final WPI_TalonFX driveMotor; 
  //Our steering motor 
  private final WPI_TalonFX steeringMotor; 
  //Our external encoder for measuring the turns of the steering motor 
  private final CANCoder steeringEncoder;

  private static final double wheelRadius = Units.inchesToMeters(4); 
  private static final double driveMotorEncoderResolution = 2048;  
  //For converting 100 milleseconds (heretofore referred to as 'ms') to seconds 
  private static final double timeConstantForConversion = 10;

  /** A simple conversion formula to turn encoder velocity (sensor units/100ms) to meters per second. 
  To do: add gear reduction to this formula. Our encoder for the drive motor is located above
  the gear reduction for our swerve module. Therefore, the gear reduction of the swerve module must be 
  factored in to our calculation. Currently, it is not. */ 
  private static final double velocityMeters = wheelRadius * 2 * Math.PI * timeConstantForConversion / driveMotorEncoderResolution;
  
  // Map ID to offset default values
  private static double[] offsets = {0, 0, 0, 0};
  private static final int ENCODER_BASE = Constants.SwerveBase.azimuthFrontLeft;

  //Tell the wheel to stop controlling the sterring motor
  private boolean isNormalizeWheel = false;
  private int stopAngle;

  //I feel the constructor is pretty self-explanatory 
  public SwerveModule(int driveMotorID, int steeringMotorID, int steeringEncoderID, int stopAngle) {
    driveMotor = new WPI_TalonFX(driveMotorID); 
    steeringMotor = new WPI_TalonFX(steeringMotorID); 
    steeringEncoder = new CANCoder(steeringEncoderID);

    this.stopAngle = stopAngle;

    String prefKey = String.format("SwerveModule/Offset_%02d", steeringMotorID);
    Preferences.initDouble(prefKey, offsets[steeringMotorID-ENCODER_BASE]);
    offsets[steeringMotorID-ENCODER_BASE] =  Preferences.getDouble(prefKey, offsets[steeringMotorID-ENCODER_BASE]);
  } 
  /** Returns the current velocity and rotation angle of the swerve module (in meters per second and 
  radians respectively) */
  public SwerveModuleState getState() { 
    return new SwerveModuleState(getVelocityMetersPerSecond(), new Rotation2d(getAngleNormalized())); 
  } 
  /** Allows us to command the swervemodule to any given veloctiy and angle, ultimately coming from our
  joystick inputs. */
  public void setDesiredState(SwerveModuleState desiredState) {
      if(isNormalizeWheel)
        return; 
      //Later, we will create a SwerveModuleState from joystick inputs to use as our desiredState
      // SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(getAngleNormalized())); 
      SwerveModuleState state = desiredState;      
       /*We need this value back in sensor units/100ms to command the falcon drive motor Note: I do not know
       why the two doubles below need to have 'final' access modifiers*/
       //final double driveOutput =  state.speedMetersPerSecond / velocityMeters;
       //We are using speedMetersPerSecond as a percent voltage value from -1 to 1 
       double percentVoltage = state.speedMetersPerSecond; 

      //  final double normalized = getAngleNormalized();
      final double absolute = getAngle();
      double delta = AngleDelta( absolute - offsets[steeringMotor.getDeviceID()-ENCODER_BASE], percentVoltage != 0 ? state.angle.getDegrees() : stopAngle );

      if (delta > 90.0) {
        delta -= 180.0 ;
        percentVoltage *= -1;
      } else if (delta < -90.0){
        delta += 180.0 ;
        percentVoltage *= -1;
      } 
      
      final double target = AngleToEncoder(absolute + delta);
      
      //Now we can command the steering motor and drive motor 
      steeringMotor.set(ControlMode.MotionMagic, target); 
      driveMotor.set(ControlMode.PercentOutput, percentVoltage); 
       
  }
  //A getter for the velocity of the drive motor, converted to meters per second.
  public double getVelocityMetersPerSecond(){ 
    return driveMotor.getSelectedSensorVelocity() * velocityMeters;
  } 

  public double getAngle(){ 
    return steeringEncoder.getPosition();
  } 
  public double getAngleNormalized(){
    return Math.IEEEremainder(steeringEncoder.getPosition(), 180.0);
  } 

  protected final static int ENCODER_COUNT = 4096;  
  public static int AngleToEncoder(double deg){
      return (int)((double)deg / 360.0 * (double)ENCODER_COUNT);
  }
  public static double EncoderToAngle(int tick){
      return tick / (double)ENCODER_COUNT * 360.0;
  }

  public static double AngleDelta(double current, double target){
    if (current < 0) current += 360.0;
    if (target < 0) target += 360.0;
    double deltaPos = current - target;
    double deltaNeg = target - current;
    if (Math.abs(deltaPos) < Math.abs(deltaNeg))
        return Math.IEEEremainder(deltaPos,360);
    else
        return Math.IEEEremainder(deltaNeg,360);
  }

  public void NormolizeModule(boolean isFinished) {
    isNormalizeWheel = !isFinished;
    steeringMotor.set(ControlMode.PercentOutput, 0);

    offsets[steeringMotor.getDeviceID()-ENCODER_BASE] = getAngle();

    if(isFinished) {
      String prefKey = String.format("SwerveModule/Offset_%02d", steeringMotor.getDeviceID());
      Preferences.setDouble(prefKey, offsets[steeringMotor.getDeviceID()-ENCODER_BASE]);
    }
  }
}
