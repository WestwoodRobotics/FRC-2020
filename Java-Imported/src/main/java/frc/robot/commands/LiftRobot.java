/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import static frc.robot.Constants.LiftConstants.*;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Lift;


/**
 * Command that is responsible for pulling up the robot
 */

public class LiftRobot extends CommandBase {

  //--------------------------------------------------------------------------------------------------
  // Variables
  private Elevator s_elevator;
  private Lift s_lift;
  private double volts;
  private double tolerance;
  //--------------------------------------------------------------------------------------------------
  // Constructor
  public LiftRobot(Lift s_lift, Elevator s_elevator) {
    this.s_elevator = s_elevator;
    this.s_lift     = s_lift;
    this.volts      = C_LIFT_VOLTS;

    addRequirements(s_lift, s_elevator); 
  }

  //--------------------------------------------------------------------------------------------------
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_elevator.stopMotor();
  }
 
  //--------------------------------------------------------------------------------------------------
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_lift.liftVolts(volts);
  }
  
  //--------------------------------------------------------------------------------------------------
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_lift.stopMotor();
  }

  //--------------------------------------------------------------------------------------------------
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return s_elevator.isCompressed(tolerance);
  }
}
