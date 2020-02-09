/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BallShooter extends SubsystemBase {
  /**
   * Creates a new BallShooter.
   */

  //--------------------------------------------------------------------------------------------------
  //Variables/Features of ball shooter
  private CANSparkMax intakeMotor1 = new CANSparkMax(P_SHOOTER_spMAX_1, MotorType.kBrushless);
  private CANSparkMax intakeMotor2 = new CANSparkMax(P_SHOOTER_spMAX_2, MotorType.kBrushless);

  // shreyes added code kindly remove at a later date
  //private WPI_VictorSPX intakeMotor3 = new WPI_VictorSPX(IntakeConstants.P_SHOOTER_vicSPX_1);
  //private WPI_VictorSPX intakeMotor4 = new WPI_VictorSPX(IntakeConstants.P_SHOOTER_vicSPX_2);
  
  private boolean intake;
  private double speed;
  //--------------------------------------------------------------------------------------------------
  // Constructor

  public BallShooter() {
    intake = false;
    speed = 1;

    intakeMotor1.disable();
    intakeMotor2.disable();

    intakeMotor2.follow(intakeMotor1, true);
  }

  //--------------------------------------------------------------------------------------------------
  // Intake methods

  public void intakeBall(){
    if (intake == false){
      intake = true;
      intakeMotor1.set(speed);

      // shreyes added code kindly remove at a later date
      // intakeMotor3.set(1);
      // intakeMotor4.set(1);
    }
  }

  public void shootBall(){
    if(!intake){
      intakeMotor1.set(-speed);

      // shreyes added code kindly remove at a later date
      // intakeMotor3.set(-1);
      // intakeMotor4.set(-1);
    } 
  }

  public void stopBall(){
    intake = false;
    intakeMotor1.set(0);

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
