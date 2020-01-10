package frc.robot;

import static frc.robot.Constants.JoyConstants.P_OI_JOY_LEFT;
import static frc.robot.Constants.JoyConstants.P_OI_JOY_RIGHT;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.TankDrive;
import frc.robot.subsystems.DriveTrain;

public class RobotContainer{
    //--------------------------------------------------------------------------------------------------
    // Declare Robot Subsystems
    public final DriveTrain s_driveTrain;
    
    private Joystick leftJoy  = new Joystick(P_OI_JOY_LEFT);            // Declaring Joysticks
    private Joystick rightJoy = new Joystick(P_OI_JOY_RIGHT);

    //--------------------------------------------------------------------------------------------------
    // Constructor
    public RobotContainer(){
        s_driveTrain = new DriveTrain();

        //Tank Drive
        s_driveTrain.setDefaultCommand(
            new TankDrive(
                () -> leftJoy.getY(), 
                () -> rightJoy.getY(), 
                s_driveTrain)
        );

        //Configure default commands (for auton choosing), button bindings
        configureButtonBindings();
    }

    private void configureButtonBindings(){
        //Configure button bindings
        (new JoystickButton(leftJoy, 1)).whenPressed(() -> s_driveTrain.setSlow(true)).whenReleased(() -> s_driveTrain.setSlow(false));
        (new JoystickButton(rightJoy, 1)).whenPressed(() -> s_driveTrain.setSlow(true)).whenReleased(() -> s_driveTrain.setSlow(false));
    }
}