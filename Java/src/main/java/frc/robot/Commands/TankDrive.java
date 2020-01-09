/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DriveTrain;

public class TankDrive extends CommandBase {

  //--------------------------------------------------------------------------------------------------
  // Variables
  
  private final DriveTrain dt = new DriveTrain();
  private double leftSpeed;                              // Speed of the left/right motors (To be set)
  private double rightSpeed;

  //--------------------------------------------------------------------------------------------------
  // Constructor
  public TankDrive(double lspeed, double rspeed) {    
    leftSpeed = lspeed;
    rightSpeed = rspeed;
    addRequirements(dt);
  }
  
  //--------------------------------------------------------------------------------------------------
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  //--------------------------------------------------------------------------------------------------
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    leftSpeed = 
    dt.driveWheels(leftSpeed, leftSpeed);
  }

  //--------------------------------------------------------------------------------------------------
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dt.stopWheels();
  }

  //--------------------------------------------------------------------------------------------------
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
