package frc.robot;

import static frc.robot.Constants.JoyConstants.P_OI_JOY_LEFT;
import static frc.robot.Constants.JoyConstants.P_OI_JOY_RIGHT;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.commands.TankDrive;
import frc.robot.subsystems.DriveTrain;

public class RobotContainer{
    //--------------------------------------------------------------------------------------------------
    // Declare Robot Subsystems
    public final DriveTrain s_driveTrain = new DriveTrain();
    
    private Joystick leftJoy  = new Joystick(P_OI_JOY_LEFT);            // Declaring Joysticks
    private Joystick rightJoy = new Joystick(P_OI_JOY_RIGHT);

    //--------------------------------------------------------------------------------------------------
    // Constructor
    public RobotContainer(){
        //Configure default commands (for auton choosing), button bindings
        configureButtonBindings();

        //Tank Drive
        s_driveTrain.setDefaultCommand(
            new TankDrive(
                () -> leftJoy.getY(), 
                () -> rightJoy.getY(), 
                s_driveTrain)
        );
    }

    private void configureButtonBindings(){
        //Configure button bindings
    }
}