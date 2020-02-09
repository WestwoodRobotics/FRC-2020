/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants.C_TRACK_WIDTH_METERS;
import static frc.robot.Constants.DriveConstants.C_kA_turn;
import static frc.robot.Constants.DriveConstants.C_kS_turn;
import static frc.robot.Constants.DriveConstants.C_kV_turn;
import static frc.robot.Constants.DriveConstants.P_DRIVE_LEFT_follow_vicSPX;
import static frc.robot.Constants.DriveConstants.P_DRIVE_LEFT_master_vicSPX;
import static frc.robot.Constants.DriveConstants.P_DRIVE_RIGHT_follow_vicSPX;
import static frc.robot.Constants.DriveConstants.P_DRIVE_RIGHT_master_vicSPX;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;

public class DriveTrain extends SubsystemBase {
  /**
   * Creates a new DriveTrain.
   */

  //--------------------------------------------------------------------------------------------------
  //Variables/Features of Drive Train
  private WPI_TalonFX leftMaster  = new WPI_TalonFX(P_DRIVE_LEFT_master_vicSPX),
                                    rightMaster = new WPI_TalonFX(P_DRIVE_RIGHT_master_vicSPX);

  private WPI_TalonFX leftFollow  = new WPI_TalonFX(P_DRIVE_LEFT_follow_vicSPX),
                            rightFollow = new WPI_TalonFX(P_DRIVE_RIGHT_follow_vicSPX);
  
  /*private WPI_VictorSPX leftMaster  = new WPI_VictorSPX(P_DRIVE_LEFT_master_vicSPX),
                      rightMaster = new WPI_VictorSPX(P_DRIVE_RIGHT_master_vicSPX);

  private WPI_VictorSPX leftFollow  = new WPI_VictorSPX(P_DRIVE_LEFT_follow_vicSPX),
                      rightFollow = new WPI_VictorSPX(P_DRIVE_RIGHT_follow_vicSPX);
  */

  private DifferentialDrive drive = new DifferentialDrive(leftMaster, rightMaster);
 
  private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(C_kS_turn, C_kV_turn, C_kA_turn);       // kF --> to make PID easier
  private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(C_TRACK_WIDTH_METERS);
  
  private boolean slowMode = false;

  private AHRS imu = new AHRS();

  /*insert encoders*/

  //--------------------------------------------------------------------------------------------------
  // Constructor
  public DriveTrain() {
    leftMaster.setInverted(true);
    leftFollow.setInverted(true);
    rightMaster.setInverted(true);
    rightFollow.setInverted(true);

    //drive.setMaxOutput(0.25);
    
    leftFollow.follow(leftMaster);
    rightFollow.follow(rightMaster);

    //drive.setSafetyEnabled(false);
  }

  //--------------------------------------------------------------------------------------------------
  
  // Drive wheels
  public void driveWheels(double leftSpeed, double rightSpeed){
    //drive.tankDrive(leftSpeed, rightSpeed);
    leftMaster.setVoltage(leftSpeed);
    rightMaster.setVoltage(rightSpeed);
    
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
    return -imu.pidGet();
  }

  public void zeroHeading(){
    imu.reset();
  }

  public DifferentialDriveKinematics getKinematics(){
    return this.kinematics;
  }

  public SimpleMotorFeedforward getFeedForward(){
    return this.feedforward;
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
