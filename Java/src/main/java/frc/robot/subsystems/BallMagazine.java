/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.PowerCellConstants.*;

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

  private CANSparkMax magazineMotor1 = new CANSparkMax(P_MAGAZINE_spMAX_1, MotorType.kBrushless);   
  private CANSparkMax magazineMotor2 = new CANSparkMax(P_MAGAZINE_spMAX_2, MotorType.kBrushless);

  private WPI_VictorSPX preRoller = new WPI_VictorSPX(P_PREROLLER_vicSPX);

  //--------------------------------------------------------------------------------------------------
  // Constructor
  public BallMagazine() {
    magazineMotor2.follow(magazineMotor1, true);
  }

  //--------------------------------------------------------------------------------------------------
  // Intake Methods
  public void shiftBall(){
    magazineMotor1.set(-C_MAGAZINE_SPEED);
  }

  public void feedBall(){
    magazineMotor1.set(-C_MAGAZINE_SPEED);
    preRoller.set(-C_PREROLLER_SPEED);
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
    magazineMotor1.set(0);
  }

  //--------------------------------------------------------------------------------------------------
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
