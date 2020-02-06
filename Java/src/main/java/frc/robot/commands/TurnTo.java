/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import static frc.robot.Constants.DriveConstants.*;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class TurnTo extends PIDCommand {
  /**
   * Creates a new TurnTo.
   */
  public TurnTo(double targetAngle, DriveTrain dt) {
    super(
        // The controller that the command will use
        new PIDController(C_kP_turn, C_kI_turn, C_kD_turn),
        dt::getHeading,          // This should return the measurement
        targetAngle,              // This should return the setpoint (can also be a constant)
        output -> dt.turnRate(output)   // This uses the output
    );
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(dt);
  
    // Configure additional PID options by calling `getController` here.
    getController().enableContinuousInput(-180, 180);
    getController().setTolerance(2.5);
  }

  //--------------------------------------------------------------------------------------------------
  // Returns if the robot is at setpoint
  public boolean atSetpoint(){
    return getController().atSetpoint();
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return atSetpoint();
  }
}
