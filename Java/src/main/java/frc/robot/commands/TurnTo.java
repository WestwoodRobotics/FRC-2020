/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DriveConstants;
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
        new PIDController(DriveConstants.C_kTurn_P, DriveConstants.C_kTurn_I, DriveConstants.C_kTurn_D),
        
        () -> dt.getHeading(),          // This should return the measurement
        () -> targetAngle,              // This should return the setpoint (can also be a constant)
        output -> dt.turnRate(output)   // This uses the output
        
        );
        
      // Use addRequirements() here to declare subsystem dependencies.
    
    
      // Configure additional PID options by calling `getController` here.
      getController().enableContinuousInput(0, 360);
      
      
      //getController().setOutputRange(-0.37, 0.37); NO SET OUTPUT RANGE
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
