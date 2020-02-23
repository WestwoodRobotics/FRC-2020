package frc.robot;

import static frc.robot.Constants.JoyConstants.P_OI_JOY_LEFT;
import static frc.robot.Constants.JoyConstants.P_OI_JOY_RIGHT;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ElevatorMax;
import frc.robot.commands.ProfiledTurnTo;
import frc.robot.commands.RunIntake;
import frc.robot.commands.TankDrive;
import frc.robot.subsystems.BallIntake;
import frc.robot.subsystems.BallShooter;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Lift;

public class RobotContainer{
    
    //--------------------------------------------------------------------------------------------------
    // Declare Robot Subsystems
    private final DriveTrain    s_driveTrain;
    public final  BallShooter   s_ballShooter;
    public final  BallIntake    s_ballIntake;
    public final  Elevator      s_elevator;
    public final  Lift          s_lift;
    
        // Declare Commands
    private ProfiledTurnTo      a_turn2;

        // Declare JoyStick
    private Joystick leftJoy    = new Joystick(P_OI_JOY_LEFT);
    private Joystick rightJoy   = new Joystick(P_OI_JOY_RIGHT);            

    //--------------------------------------------------------------------------------------------------
    // Constructor
    public RobotContainer(){
        s_driveTrain    = new DriveTrain();
        s_ballShooter   = new BallShooter();
        s_ballIntake    = new BallIntake();
        s_elevator      = new Elevator();
        s_lift          = new Lift();
        a_turn2         = new ProfiledTurnTo(90.0, s_driveTrain);
     
        //Tank Drive
        s_driveTrain.setDefaultCommand(
            new TankDrive(
                () -> -leftJoy.getY(), 
                () -> rightJoy.getY(), 
                s_driveTrain)
        );

        //Configure default commands (for auton choosing), button bindings
        configureButtonBindings();
    }

    //--------------------------------------------------------------------------------------------------
    // Configuring Buttons

    private void configureButtonBindings(){
        //Configure button bindings
        (new JoystickButton(rightJoy, 1)).whenPressed(()    -> s_driveTrain.setSlow(true))
                                         .whenReleased(()   -> s_driveTrain.setSlow(false));

        (new JoystickButton(leftJoy, 1)).whenPressed(()     -> s_driveTrain.setSlow(true))
                                        .whenReleased(()    -> s_driveTrain.setSlow(false));

        // BallIntake Commands
        (new JoystickButton(leftJoy, 3)).whenPressed(()     -> new RunIntake(s_ballIntake));
        
        // BallShooter Commands
        
        
        // Elevator Commands
        (new JoystickButton(leftJoy, 5)).whileActiveOnce(new ElevatorMax(s_elevator));
        
        // Lift Commands
        (new JoystickButton(leftJoy, 6)).whenPressed(()     -> s_lift.liftPercentage(1))      //TODO: change speed*******************
                                        .whenReleased(()    -> s_lift.stopMotor());
    }

    public Command getAutonomousCommand(String m_autoSelected){
        s_driveTrain.zeroHeading();
        s_driveTrain.resetOdometry(new Pose2d(0.0, 0.0, new Rotation2d(0.0)));
        switch (m_autoSelected) {
            /*case "TurnTo":
                return new TurnTo(SmartDashboard.getNumber("Degrees", 0.0), s_driveTrain);*/
            case "ProfiledTurnTo":
                return new ProfiledTurnTo(SmartDashboard.getNumber("Degrees", 0.0), s_driveTrain);
            default:
                return null;
        }

    }

}