/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class BallShooter extends SubsystemBase {
  /**
   * Creates a new BallShooter.
   */

  //--------------------------------------------------------------------------------------------------
  //Variables/Features of ball shooter
  private WPI_VictorSPX intakeMotor1 = new WPI_VictorSPX(IntakeConstants.P_SHOOTER_vicSPX_1);
  private WPI_VictorSPX intakeMotor2 = new WPI_VictorSPX(IntakeConstants.P_SHOOTER_vicSPX_2);

  // shreyes added code kindly remove at a later date
  private WPI_VictorSPX intakeMotor3 = new WPI_VictorSPX(IntakeConstants.P_SHOOTER_vicSPX_1);
  private WPI_VictorSPX intakeMotor4 = new WPI_VictorSPX(IntakeConstants.P_SHOOTER_vicSPX_2);
  
  private boolean intake;
  //--------------------------------------------------------------------------------------------------
  // Constructor

  public BallShooter() {
    intake = false;
  }

  //--------------------------------------------------------------------------------------------------
  // Intake methods

  public void intakeBall(){
    if (intake == false){
      intake = true;
      intakeMotor1.set(1);
      intakeMotor2.set(1);

      // shreyes added code kindly remove at a later date
      // intakeMotor3.set(1);
      // intakeMotor4.set(1);
    }
  }

  public void shootBall(){
    if(!intake){
      intakeMotor1.set(-1);
      intakeMotor2.set(-1);

      // shreyes added code kindly remove at a later date
      // intakeMotor3.set(-1);
      // intakeMotor4.set(-1);
    } 
  }

  public void stopBall(){
    intake = false;
    intakeMotor1.set(0);
    intakeMotor2.set(0);

    // shreyes added code kindly remove at a later date
   // intakeMotor3.set(0);
      // intakeMotor4.set(0);
  }

  //--------------------------------------------------------------------------------------------------
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
