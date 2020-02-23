/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import static frc.robot.Constants.LiftConstants.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
   * This subsystem is responsible for the actual lifting of 
   * the robot
   */
  
public class Lift extends SubsystemBase {
    
  //--------------------------------------------------------------------------------------------------
  // Variables/Features of Lift
  private final WPI_TalonSRX liftMotor1   = new WPI_TalonSRX(P_LIFT_motor1_talSRX),
                             liftMotor2   = new WPI_TalonSRX(P_LIFT_motor2_talSRX);

  //--------------------------------------------------------------------------------------------------
  // Constructor
  public Lift() {
    liftMotor2.follow(liftMotor1);
  }

  //--------------------------------------------------------------------------------------------------
  // Method of the lift
  public void liftVoltage(double volts){
    liftMotor1.setVoltage(volts);
  }

  public void liftPercentage(double speed){
    liftMotor1.set(speed);
  }

  public void stopMotor(){
    liftMotor1.set(0);
  }

  //--------------------------------------------------------------------------------------------------
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
