/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class TankDrive extends CommandBase {

  //--------------------------------------------------------------------------------------------------
  // Variables
  private DriveTrain s_dt;
  private DoubleSupplier leftSpeed;
  private DoubleSupplier rightSpeed;
  

  //--------------------------------------------------------------------------------------------------
  // Constructor
  public TankDrive(DoubleSupplier leftSpeed, DoubleSupplier rightSpeed, DriveTrain s_dt) {    
    this.s_dt = s_dt;
    this.leftSpeed = leftSpeed;
    this.rightSpeed = rightSpeed;
    addRequirements(s_dt);
  }
  
  //--------------------------------------------------------------------------------------------------
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_dt.stopWheels();
  }

  //--------------------------------------------------------------------------------------------------
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!s_dt.getSlow())
      s_dt.driveWheelsPercent(leftSpeed.getAsDouble(), rightSpeed.getAsDouble());
    else
      s_dt.driveWheelsPercent(leftSpeed.getAsDouble()/2.0, rightSpeed.getAsDouble()/2.0);
  }

  //--------------------------------------------------------------------------------------------------
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_dt.stopWheels();
  }

  //--------------------------------------------------------------------------------------------------
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
