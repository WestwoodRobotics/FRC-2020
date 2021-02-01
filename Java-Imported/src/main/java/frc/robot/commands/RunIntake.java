/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.BallIntake;
import frc.robot.subsystems.BallMagazine;

public class RunIntake extends CommandBase {
  /**
   * Creates a new RunIntake.
   */

  BallIntake s_ballIntake;
  BallMagazine s_ballMagazine;

  public RunIntake(BallIntake s_ballIntake, BallMagazine s_ballMagazine) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_ballIntake = s_ballIntake;
    this.s_ballMagazine = s_ballMagazine;
    addRequirements(s_ballIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_ballIntake.extend();
    s_ballMagazine.stopPreroller();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_ballIntake.intakeBall();
    s_ballMagazine.shiftBall();
    s_ballMagazine.stopPreroller();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_ballIntake.contract();
    s_ballIntake.stopBall();
    s_ballMagazine.stopMagazine();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
