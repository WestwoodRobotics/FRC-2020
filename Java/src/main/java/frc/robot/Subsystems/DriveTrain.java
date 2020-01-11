/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

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

  /*insert encoders*/

  //--------------------------------------------------------------------------------------------------
  // Constructor
  public DriveTrain() {
    leftFollow.follow(leftMaster);
    rightFollow.follow(rightMaster);

    leftMaster.setInverted(true);
    leftFollow.setInverted(true);
    rightMaster.setInverted(true);
    rightFollow.setInverted(true);
  }

  //--------------------------------------------------------------------------------------------------
  
  // Drive wheels
  public void driveWheels(double leftSpeed, double rightSpeed){
    drive.tankDrive(leftSpeed, rightSpeed);
    //System.out.println("Working");
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
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
