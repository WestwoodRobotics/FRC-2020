/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;
import static frc.robot.Constants.LiftConstants.*;


/**
 * Command that is responsible for raising the elevator to the max point
 * and deploying the hooks
 */

public class ElevatorMax extends CommandBase {

  //--------------------------------------------------------------------------------------------------
  // Variables
  private Elevator s_elevator;
  private double volts;
  private double tolerance;
  //--------------------------------------------------------------------------------------------------
  // Constructor
  public ElevatorMax(Elevator s_elevator) {
    this.s_elevator = s_elevator;
    
    tolerance = C_ELEVATOR_MAX_TOLERANCE;
    volts = C_ELEVATOR_MAX_VOLTS;

    addRequirements(s_elevator); 
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
    s_elevator.eleLiftVolts(volts);
  }
  
  //--------------------------------------------------------------------------------------------------
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_elevator.stall();
  }

  //--------------------------------------------------------------------------------------------------
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return s_elevator.atMax(tolerance);
  }
}
