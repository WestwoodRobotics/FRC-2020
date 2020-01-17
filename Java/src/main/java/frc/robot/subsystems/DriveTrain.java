/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveTrain extends SubsystemBase {
  /**
   * Creates a new DriveTrain.
   */

  //--------------------------------------------------------------------------------------------------
  //Variables/Features of Drive Train

  private WPI_VictorSPX leftMaster  = new WPI_VictorSPX(DriveConstants.P_DRIVE_LEFT_vicSPX_1),
                            rightMaster = new WPI_VictorSPX(DriveConstants.P_DRIVE_RIGHT_vicSPX_1);

  private WPI_VictorSPX leftFollow  = new WPI_VictorSPX(DriveConstants.P_DRIVE_LEFT_vicSPX_2),
                            rightFollow = new WPI_VictorSPX(DriveConstants.P_DRIVE_RIGHT_vicSPX_2);

  private DifferentialDrive drive = new DifferentialDrive(leftMaster, rightMaster);
  
  private boolean slowMode = false;

  private AHRS m_gyro = new AHRS();

  /*insert encoders*/

  //--------------------------------------------------------------------------------------------------
  // Constructor
  public DriveTrain() {
    leftMaster.configFactoryDefault();
    leftFollow.configFactoryDefault();
    rightMaster.configFactoryDefault();
    rightFollow.configFactoryDefault();

    leftMaster.setInverted(true);
    leftFollow.setInverted(true);
    rightMaster.setInverted(true);
    rightFollow.setInverted(true);
    
    leftFollow.follow(leftMaster);
    rightFollow.follow(rightMaster);

    //drive.setSafetyEnabled(false);
  }

  //--------------------------------------------------------------------------------------------------
  
  // Drive wheels
  public void driveWheels(double leftSpeed, double rightSpeed){
    drive.tankDrive(leftSpeed, rightSpeed);
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

  public float getHeading(){
    return m_gyro.getCompassHeading();
  }

  // Setting the gyro value to 0
  public void zeroHeading(){
    m_gyro.reset();
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
