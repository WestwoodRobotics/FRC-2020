/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallIntake;

public class RunIntake extends CommandBase {
  /**
   * Creates a new RunIntake.
   */

  BallIntake s_ballIntake;

  public RunIntake(BallIntake s_ballIntake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_ballIntake = s_ballIntake;
    addRequirements(s_ballIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_ballIntake.extend();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_ballIntake.intakeBall();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_ballIntake.contract();
    s_ballIntake.stopBall();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
