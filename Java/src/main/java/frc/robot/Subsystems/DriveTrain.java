/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
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

  //Grouping the master motor with its follow motors
  private SpeedControllerGroup lMotorGroup = new SpeedControllerGroup(leftMaster, leftFollow),
                               rMotorGroup = new SpeedControllerGroup(rightMaster, rightFollow);

  public DifferentialDrive d = new DifferentialDrive(lMotorGroup, rMotorGroup);
  

  /*insert encoders*/

  //--------------------------------------------------------------------------------------------------
  // Constructor
  public DriveTrain() {
    this.setDefaultCommand(new TankDrive());
  }

                /*Here*/



  //--------------------------------------------------------------------------------------------------
  // Method for driving the wheels
  public void driveWheels(double lSpeed, double rSpeed){
    d.tankDrive(lSpeed, rSpeed);
  }

  // Method for stopping the wheels
  public void stopWheels(){
    d.tankDrive(0, 0);
  }

  //--------------------------------------------------------------------------------------------------
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
