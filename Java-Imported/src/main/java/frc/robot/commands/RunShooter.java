/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import static frc.robot.Constants.PowerCellConstants.*;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallMagazine;
import frc.robot.subsystems.BallShooter;

public class RunShooter extends CommandBase {
  /**
   * Creates a new RunShooter.
   */
  BallShooter s_ballShooter;
  BallMagazine s_ballMagazine;
  E_SHOOT_POS pos;

  double speed;
  boolean feedBalls;


  public RunShooter(E_SHOOT_POS pos, BallShooter s_ballShooter, BallMagazine s_ballMagazine) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_ballShooter = s_ballShooter;
    this.s_ballMagazine = s_ballMagazine;
    this.pos = pos;

    feedBalls = false;

    addRequirements(s_ballShooter);
    addRequirements(s_ballMagazine);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //s_ballShooter.setHood(pos);
    s_ballShooter.setShooterVelocityPID(pos);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_ballShooter.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public void setPos(E_SHOOT_POS pos){
    this.pos = pos;
  }
}
