/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.LiftConstants.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;
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
  private boolean isCompressed = false;

  //--------------------------------------------------------------------------------------------------
  // Constructor
  public Elevator() {
    this.zeroEncoder();
  }

  //--------------------------------------------------------------------------------------------------
  // Methods for Elevator

  public void eleLiftVolts(double v)        {elevatorMotor.setVoltage(v);}
  public void eleLiftPercent(double speed)  {elevatorMotor.set(speed);}
  public void stopMotor()                   {elevatorMotor.stopMotor();}

  public void coast(){
    this.elevatorMotor.setNeutralMode(NeutralMode.Coast);
  }

  public void stall(){
    this.eleLiftVolts(C_ELEVATOR_stall_VOLT);
  }

  public void lowerElevator(){
    this.eleLiftVolts(C_ELEVATOR_lower_slow_VOLT);
  }
  //-------------------------------------------------------------
      //Encoders
  public double getEncoderPos()   {return elevatorMotor.getSelectedSensorPosition();}
  public double getEncoderVel()   {return elevatorMotor.getSelectedSensorVelocity();}
  
  public void   zeroEncoder()     {elevatorMotor.setSelectedSensorPosition(0);}

  //-------------------------------------------------------------
      // Velocity
  public double getVelocityMeterPerSec()  {return talVelToMetersPerSec(getEncoderVel());} // TODO: Complete the method**************
  

  //-------------------------------------------------------------
  public boolean isCompressed(double tolerance){
    if(this.getEncoderPos() == tolerance || this.getEncoderPos() == -tolerance){
      isCompressed = true;
    }
    return isCompressed;
  }
  
  public boolean atMax(double tolerance)  {return getVelocityMeterPerSec() <= tolerance;}


  //--------------------------------------------------------------------------------------------------
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
