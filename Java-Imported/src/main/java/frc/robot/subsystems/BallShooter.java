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
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This subsystem is responsible for shooting the PowerCells
 */

public class BallShooter extends SubsystemBase {
  
  //--------------------------------------------------------------------------------------------------
  //Variables/Features of BallShooter
  private CANSparkMax shooterMotor1;
  private CANSparkMax shooterMotor2;

  //private Solenoid hood = new Solenoid(P_HOOD_sol);

  private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(C_kS, C_kV, C_kA);
  private PIDController velPID = new PIDController(C_kP, C_kI, C_kD);

  private double speedSetpoint = 0.0; // Rotations per minute
  private E_SHOOT_POS pos;
  //--------------------------------------------------------------------------------------------------
  // Constructor

  public BallShooter() {
    //intake = false;
    shooterMotor1 = new CANSparkMax(P_SHOOTER_spMAX_1, MotorType.kBrushless);
    shooterMotor2 = new CANSparkMax(P_SHOOTER_spMAX_2, MotorType.kBrushless);

    shooterMotor1.restoreFactoryDefaults();
    shooterMotor2.restoreFactoryDefaults();

    shooterMotor1.setIdleMode(IdleMode.kCoast);
    shooterMotor2.setIdleMode(IdleMode.kCoast);
    
    shooterMotor1.setInverted(true);
    shooterMotor2.follow(shooterMotor1, true);

    pos = E_SHOOT_POS.CLOSE;
    speedSetpoint = 0.0;
  }

  //--------------------------------------------------------------------------------------------------
  // Shooter Methods
  public void setShooterPercent(double percent){
    shooterMotor1.set(percent);
  }

  public void setShooterVoltage(double voltage){
    //shooterMotor2.disable();
    shooterMotor1.setVoltage(voltage);
    //System.out.println(voltage);
  }

  public void setShooterVelocityPID(double rotationsPerMin){
    speedSetpoint = rotationsPerMin;
    double volts = 0.0;

    volts += feedforward.calculate(rotationsPerMin/60.0);
    volts += velPID.calculate(getShooterVel()/60.0, rotationsPerMin/60.0);

    System.out.println(getShooterVel());

    this.setShooterVoltage(volts);
  }

  public void setShooterVelocityPID(E_SHOOT_POS pos){
    if(pos == E_SHOOT_POS.CLOSE)
      this.setShooterVelocityPID(C_SHOOTER_SPEED_CLOSE);
    else if(pos == E_SHOOT_POS.TRENCH)
      this.setShooterVelocityPID(C_SHOOTER_SPEED_TRENCH);
    else
      this.stopShooter();
  }

  public void setHood(E_SHOOT_POS pos){
    if(pos == E_SHOOT_POS.CLOSE){
      //hood.set(false);
    }
    else if(pos == E_SHOOT_POS.TRENCH){
      //hood.set(true);
    }
  }

  public double getShooterVel(){
    return shooterMotor1.getEncoder().getVelocity();
  }

  public boolean isFlywheelReady(){
    if(Math.abs(this.getShooterVel() - this.speedSetpoint) < C_SHOOTER_SPEED_TOLERANCE){
      return true;
    }
    return false;
  }

  public void stopShooter(){
    shooterMotor1.stopMotor();
  }

  //--------------------------------------------------------------------------------------------------
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
