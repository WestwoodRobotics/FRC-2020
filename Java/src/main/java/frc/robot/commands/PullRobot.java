/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Lift;

public class PullRobot extends CommandBase {
  /**
   * Creates a new PullRobot.
   */

  //--------------------------------------------------------------------------------------------------
  // Variables
  private Lift s_lift;
  
  //--------------------------------------------------------------------------------------------------
  // Constructor
  public PullRobot(Lift s_lift) {
    this.s_lift = s_lift;

    addRequirements(s_lift); 
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
    /*TODO: s_lift.liftPercent(##);
    s_lift.liftVoltage(##);
    */
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
    return false;
  }
}
