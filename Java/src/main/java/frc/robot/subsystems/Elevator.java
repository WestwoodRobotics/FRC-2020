/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.LiftConstants.*;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  /**
   * This subsystem is only responsible for deploying the hooks 
   * but not the actual pulling 
   */

  //--------------------------------------------------------------------------------------------------
  // Variables/Features of Lift
  private final WPI_TalonSRX elevatorMotor = new WPI_TalonSRX(P_ELEVATOR_motor_talSRX);    

  //--------------------------------------------------------------------------------------------------
  // Constructor

  public Elevator() {
  }

  //--------------------------------------------------------------------------------------------------
  // Methods for Elevator

  public void eleLiftVolts(double v){
    elevatorMotor.setVoltage(v);
  }

  public void eleLiftPercent(double speed){
    elevatorMotor.set(speed);
  }

  public void stopMotor(){
    elevatorMotor.set(0.1); //TODO: Change this later
  }
  
  public double getVelocity(){
    return talVelToMetersPerSec(elevatorMotor.getSelectedSensorVelocity());   // TODO: Complete the method**************
  }

  public boolean atMax(){
    return getVelocity() > 0.1;                        //TODO: Check the actual velocity********************************
  }


  //--------------------------------------------------------------------------------------------------
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
