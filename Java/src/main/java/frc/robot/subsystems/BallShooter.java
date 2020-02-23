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

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This subsystem is responsible for shooting the PowerCells
 */

public class BallShooter extends SubsystemBase {
  
  //--------------------------------------------------------------------------------------------------
  //Variables/Features of BallShooter
  private CANSparkMax shooterMotor1 = new CANSparkMax(P_SHOOTER_spMAX_1, MotorType.kBrushless);
  private CANSparkMax shooterMotor2 = new CANSparkMax(P_SHOOTER_spMAX_2, MotorType.kBrushless);

  private WPI_VictorSPX preRoller = new WPI_VictorSPX(P_PREROLLER_vicSPX);

  private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(C_kS, C_kV, C_kA);
  private PIDController velPID = new PIDController(C_kP, C_kI, C_kD);
  //--------------------------------------------------------------------------------------------------
  // Constructor

  public BallShooter() {
    //intake = false;
    shooterMotor2.follow(shooterMotor1, true);
  }

  //--------------------------------------------------------------------------------------------------
  // Shooter Methods
  public void setShooterPercent(double percent){
    shooterMotor1.set(percent);
  }

  public void setShooterVoltage(double voltage){
    shooterMotor1.setVoltage(voltage);
  }

  public void setShooterVelocityPID(double metersPerSec){
    double volts = 0.0;

    volts += feedforward.calculate(metersPerSec);
    volts += velPID.calculate(getShooterVel());

    this.setShooterVoltage(volts);
  }

  // TODO: Eventually change this method to a set constant speed
  public void setPrerollerPercent(double percent){
    preRoller.set(percent);
  }

  public void setPrerollerVoltage(double voltage){
    preRoller.setVoltage(voltage);
  }

  public double getShooterVel(){
    return shooterMotor1.getEncoder().getVelocity();
  }

  public void stopShooter(){
    shooterMotor1.stopMotor();
  }

  public void stopPreroller(){
    preRoller.stopMotor();
  }

  //--------------------------------------------------------------------------------------------------
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
