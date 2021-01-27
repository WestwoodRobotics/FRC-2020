/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class DriveDistTrapezoidProf extends TrapezoidProfileCommand {
  /**
   * Creates a new DriveDistTrapezoidProf.
   */
  public DriveDistTrapezoidProf(double meters, DriveTrain s_driveTrain) {
    super(
        // The motion profile to be executed
        new TrapezoidProfile(
            // The motion profile constraints
            new TrapezoidProfile.Constraints(1, 1),
            // Goal state
            new TrapezoidProfile.State(meters, 0),
            // Initial state
            new TrapezoidProfile.State(0, 0)),
        state -> {
          // Use current trajectory state here
          //System.out.println("Pos: " + state.position);
          if(state.velocity > 1){
            System.out.println("Vel: " + 1);
            s_driveTrain.setVelocityPID(1, 1);
          }
          else{
            System.out.println("Vel: " + state.velocity);
            s_driveTrain.setVelocityPID(state.velocity, state.velocity);
          }
        }
    );

    addRequirements(s_driveTrain);
  }
}
