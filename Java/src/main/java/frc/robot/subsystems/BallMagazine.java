/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.PowerCellConstants.C_MAGAZINE_SPEED;
import static frc.robot.Constants.PowerCellConstants.P_MAGAZINE_spMAX_1;
import static frc.robot.Constants.PowerCellConstants.P_MAGAZINE_spMAX_2;

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

  //--------------------------------------------------------------------------------------------------
  // Constructor
  public BallMagazine() {
    magazineMotor2.follow(magazineMotor1, true);
  }

  //--------------------------------------------------------------------------------------------------
  // Intake Methods
  public void feedBall(){
    magazineMotor1.set(-C_MAGAZINE_SPEED);
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
