/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.LiftConstants.*;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
   * This subsystem is responsible for the actual lifting of 
   * the robot
   */
  
public class Lift extends SubsystemBase {
    
  //--------------------------------------------------------------------------------------------------
  // Variables/Features of Lift
  private final WPI_VictorSPX liftMotor1   = new WPI_VictorSPX(P_LIFT_motor1_vicSPX);
  private final WPI_VictorSPX    liftMotor2   = new WPI_VictorSPX(P_LIFT_motor2_vicSPX);

  //--------------------------------------------------------------------------------------------------
  // Constructor
  public Lift() {
    liftMotor1.setInverted(true);
    liftMotor2.follow(liftMotor1);
  }

  //--------------------------------------------------------------------------------------------------
  // Method of the lift
public void liftVolts(double volts)    {liftMotor1.setVoltage(volts);}
public void liftPercent(double speed)  {liftMotor1.set(speed);}
public void stopMotor()                {liftMotor1.stopMotor();}
   
  //--------------------------------------------------------------------------------------------------
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
