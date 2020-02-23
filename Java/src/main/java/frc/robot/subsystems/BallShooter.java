/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.PowerCellConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This subsystem is responsible for shooting the PowerCells
 */

public class BallShooter extends SubsystemBase {
  
  //--------------------------------------------------------------------------------------------------
  //Variables/Features of BallShooter
  private CANSparkMax shooterMotor1 = new CANSparkMax(P_SHOOTER_spMAX_1, MotorType.kBrushless);
  private CANSparkMax shooterMotor2 = new CANSparkMax(P_SHOOTER_spMAX_2, MotorType.kBrushless);

  private double speed;

  /** shreyes added code kindly remove at a later date
   *    private WPI_VictorSPX intakeMotor3 = new WPI_VictorSPX(IntakeConstants.P_SHOOTER_vicSPX_1);
   *    private WPI_VictorSPX intakeMotor4 = new WPI_VictorSPX(IntakeConstants.P_SHOOTER_vicSPX_2);
   */
  
  //private boolean intake;                                                             TODO: Delete...?*********************

  //--------------------------------------------------------------------------------------------------
  // Constructor

  public BallShooter() {
    //intake = false;
    speed = C_SHOOTER_SPEED;
    shooterMotor2.follow(shooterMotor1, true);
  }

  //--------------------------------------------------------------------------------------------------
  // Shooter Methods

  public void shootBall(){
      //if(!intake){
    shooterMotor1.set(-speed);

      /** shreyes added code kindly remove at a later date
       * intakeMotor3.set(-1);
       * intakeMotor4.set(-1);
       */
      
      //} "if" bracket
  }

  public void stopBall(){
      //intake = false;
    shooterMotor1.set(0);

      /** shreyes added code kindly remove at a later date
       *  intakeMotor3.set(0);
       *  intakeMotor4.set(0);
       */
  }

  //--------------------------------------------------------------------------------------------------
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
