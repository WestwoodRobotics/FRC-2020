/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants.P_DRIVE_LEFT_follow_vicSPX;
import static frc.robot.Constants.DriveConstants.P_DRIVE_LEFT_master_vicSPX;
import static frc.robot.Constants.DriveConstants.P_DRIVE_RIGHT_follow_vicSPX;
import static frc.robot.Constants.DriveConstants.P_DRIVE_RIGHT_master_vicSPX;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
  /**
   * Creates a new DriveTrain.
   */

  //--------------------------------------------------------------------------------------------------
  //Variables/Features of Drive Train
  private WPI_VictorSPX leftMaster  = new WPI_VictorSPX(P_DRIVE_LEFT_master_vicSPX),
                                    rightMaster = new WPI_VictorSPX(P_DRIVE_RIGHT_master_vicSPX);

  private WPI_VictorSPX leftFollow  = new WPI_VictorSPX(P_DRIVE_LEFT_follow_vicSPX),
                            rightFollow = new WPI_VictorSPX(P_DRIVE_RIGHT_follow_vicSPX);

  private DifferentialDrive drive = new DifferentialDrive(leftMaster, rightMaster);
  
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

    drive.setMaxOutput(1);
    
    //leftFollow.follow(leftMaster);
    //rightFollow.follow(rightMaster);

    //drive.setSafetyEnabled(false);
  }

  //--------------------------------------------------------------------------------------------------
  
  // Drive wheels
  public void driveWheels(double leftSpeed, double rightSpeed){
    drive.tankDrive(leftSpeed, rightSpeed);
    //leftMaster.set(0.5);
    
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
  // Gyro / Turning DriveTrain
  public double getHeading(){
    SmartDashboard.putNumber("gyro", imu.pidGet());
    return imu.pidGet();
  }

  // Setting the gyro value to 0
  public void zeroHeading(){
    imu.reset();
  }

  public void turnRate(double rt){
    drive.curvatureDrive(0, rt, true);
  }

  //--------------------------------------------------------------------------------------------------
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
