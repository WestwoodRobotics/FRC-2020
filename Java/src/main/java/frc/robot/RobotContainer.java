package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Commands.TankDrive;
import frc.robot.Constants.OIConstants;
import frc.robot.Subsystems.DriveTrain;

public class RobotContainer{
    //--------------------------------------------------------------------------------------------------
    // Declare Robot Subsystems
    private final DriveTrain dt = new DriveTrain();

    
    Joystick leftJoy  = new Joystick(OIConstants.P_OI_JOY_LEFT),            // Declaring Joystics
             rightJoy = new Joystick(OIConstants.P_OI_JOY_RIGHT);

    //--------------------------------------------------------------------------------------------------
    // Constructor
    public RobotContainer(){
        //Configure default commands (for auton choosing), button bindings
        configureButtonBindings();

    }

    private void configureButtonBindings(){
        //Configure button bindings
    }
}