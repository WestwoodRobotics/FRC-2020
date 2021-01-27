/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.PowerCellConstants.*;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This subsystem is responsible for intaking the PowerCells to the
 * BallMagazine
 */

public class BallIntake extends SubsystemBase {
  
  //--------------------------------------------------------------------------------------------------
  //Variables/Features of BallIntake                                         // TODO: Change motor types************************

  private WPI_TalonSRX intakeMotor1 = new WPI_TalonSRX(P_INTAKE_talSRX_1);
  
  private Solenoid intakeSol1 = new Solenoid(P_INTAKE_sol_1);
  //private Solenoid intakeSol2 = new Solenoid(P_INTAKE_sol_2);

  //--------------------------------------------------------------------------------------------------
  // Constructor
  public BallIntake() {
    intakeSol1.set(false);
    //intakeSol2.set(false);
    intakeMotor1.setInverted(true);
    
  }

  //--------------------------------------------------------------------------------------------------
  // Intake Methods
  public void intakeBall(){
    intakeMotor1.set(C_INTAKE_SPEED);
  }

  public void extend(){
    intakeSol1.set(true);
    //intakeSol2.set(true);
  }

  public void contract(){
    intakeSol1.set(false);
    //intakeSol2.set(false);
  }

  public void stopBall(){
    intakeMotor1.stopMotor();
  }

  //--------------------------------------------------------------------------------------------------
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
