/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.PowerCellConstants.*;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This subsystem is responsible for delivering the PowerCells to 
 * the BallShooter
 */

public class BallMagazine extends SubsystemBase {
  
  //--------------------------------------------------------------------------------------------------
  //Variables/Features of BallIntake                                      // TODO: Change motor types************************

  private WPI_TalonSRX magazineMotor1 = new WPI_TalonSRX(P_MAGAZINE_talSRX_1);

  private WPI_TalonSRX preRoller = new WPI_TalonSRX(P_PREROLLER_talSRX);

  //--------------------------------------------------------------------------------------------------
  // Constructor
  public BallMagazine() {
    magazineMotor1.setInverted(true);
    preRoller.setInverted(true);
  }

  //--------------------------------------------------------------------------------------------------
  // Intake Methods
  public void shiftBall(){
    magazineMotor1.set(C_MAGAZINE_PERCENT);
  }

  public void feedBall(){
    magazineMotor1.set(C_MAGAZINE_PERCENT);
    preRoller.set(C_PREROLLER_SPEED);
  }

  // TODO: Eventually change this method to a set constant speed
  public void setPrerollerPercent(double percent){
    preRoller.set(percent);
  }

  public void setPrerollerVoltage(double voltage){
    preRoller.setVoltage(voltage);
  }
  
  public void stopPreroller(){
    preRoller.stopMotor();
  }

  public void stopMagazine(){
    magazineMotor1.stopMotor();
  }

  //--------------------------------------------------------------------------------------------------
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
