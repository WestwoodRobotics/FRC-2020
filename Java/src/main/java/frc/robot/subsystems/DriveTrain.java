/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants.*;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
  /**
   * Creates a new DriveTrain.
   */

  //--------------------------------------------------------------------------------------------------
  //Variables/Features of Drive Train
  private WPI_TalonFX leftMaster  = new WPI_TalonFX(P_DRIVE_LEFT_master_talFX),
                                    rightMaster = new WPI_TalonFX(P_DRIVE_RIGHT_master_talFX);

  private WPI_TalonFX leftFollow  = new WPI_TalonFX(P_DRIVE_LEFT_follow_talFX),
                            rightFollow = new WPI_TalonFX(P_DRIVE_RIGHT_follow_talFX);
  
                            
  /*private WPI_VictorSPX leftMaster  = new WPI_VictorSPX(P_DRIVE_LEFT_master_talFX),
                      rightMaster = new WPI_VictorSPX(P_DRIVE_RIGHT_master_talFX);

  private WPI_VictorSPX leftFollow  = new WPI_VictorSPX(P_DRIVE_LEFT_follow_talFX),
                      rightFollow = new WPI_VictorSPX(P_DRIVE_RIGHT_follow_talFX);*/
  

  private PIDController leftVelPID = new PIDController(C_kP_LEFT, C_kI_LEFT, C_kD_LEFT);
  private PIDController rightVelPID = new PIDController(C_kP_RIGHT, C_kI_RIGHT, C_kD_RIGHT);

  private DifferentialDrive drive = new DifferentialDrive(leftMaster, rightMaster);
 
  private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(C_kS, C_kV, C_kA);
  private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(C_TRACK_WIDTH_METERS);
  
  private boolean slowMode = false;

  private AHRS imu = new AHRS();

  /*insert encoders*/

  //--------------------------------------------------------------------------------------------------
  // Constructor
  public DriveTrain() {
    //leftMaster.setInverted(true);
    //leftFollow.setInverted(true);
    rightMaster.setInverted(true);
    rightFollow.setInverted(true);
    
    leftFollow.follow(leftMaster);
    rightFollow.follow(rightMaster);

    //drive.setSafetyEnabled(false);
  }

  //--------------------------------------------------------------------------------------------------
  
  // Drive wheels (percent) [-1.0, 1.0]
  public void driveWheelsPercent(double leftPercent, double rightPercent){
    drive.tankDrive(leftPercent, rightPercent);
  }

  public void driveWheelsVolts(double leftVolts, double rightVolts){
    leftMaster.setVoltage(leftVolts);
    rightMaster.setVoltage(rightVolts);
  }

  public void setVelocityPID(double leftMetersPerSec, double rightMetersPerSec){
    double leftVolts = 0.0;
    double rightVolts = 0.0;

    DifferentialDriveWheelSpeeds wheelSpeeds = this.getWheelSpeeds();

    leftVolts += feedforward.calculate(leftMetersPerSec);
    rightVolts += feedforward.calculate(rightMetersPerSec);

    SmartDashboard.putNumber("left speed", wheelSpeeds.leftMetersPerSecond);
    SmartDashboard.putNumber("right speed", wheelSpeeds.rightMetersPerSecond);

    leftVolts += leftVelPID.calculate(wheelSpeeds.leftMetersPerSecond, leftMetersPerSec);
    rightVolts += rightVelPID.calculate(wheelSpeeds.rightMetersPerSecond, rightMetersPerSec);

    System.out.println(leftVolts + ", " + rightVolts);

    this.driveWheelsVolts(leftVolts, rightVolts);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds(this.falconVelToMetersPerSec(leftMaster.getSelectedSensorVelocity()), this.falconVelToMetersPerSec(rightMaster.getSelectedSensorVelocity()));
  }

  public double falconVelToMetersPerSec(double ticksPerDecasec){
    double metersPerSec = ticksPerDecasec*10.0*Math.PI*C_WHEEL_DIAMETER_METERS/C_EPR;
    return metersPerSec;
  }



  //Set slow mode
  public void setSlow(boolean slowMode){
    this.slowMode = slowMode;
  }

  //Get slow mode
  public boolean getSlow(){
    return slowMode;
  }

  // Stop wheels
  public void stopWheels(){
    drive.stopMotor();
  }

  //--------------------------------------------------------------------------------------------------
  // Turn to Angle
  public double getHeading(){
    return imu.pidGet();
  }

  public void zeroHeading(){
    imu.reset();
  }

  public DifferentialDriveKinematics getKinematics(){
    return this.kinematics;
  }

  public SimpleMotorFeedforward getFeedForward(String leftOrRight){
    return feedforward;
  }
  //--------------------------------------------------------------------------------------------------

  public void turnRate(double rt){
    drive.curvatureDrive(0, rt, true);
  }

  public double leftEncoderGet(){
    return leftMaster.getSelectedSensorPosition();
  }

  public double rightEncoderGet(){
    return rightMaster.getSelectedSensorPosition();
  }

  //--------------------------------------------------------------------------------------------------
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
