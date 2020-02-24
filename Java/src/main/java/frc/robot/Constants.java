package frc.robot;

// Create a new "constants" class for each mechanism/subsystem
public final class Constants{
    
    //--------------------------------------------------------------------------------------------------
    // Constants for the DriveTrain
    // All units in meters and meters per second

    public static final class DriveConstants{
        
        // Motor Controller Ports
        public static final int P_DRIVE_LEFT_master_talFX = 0,
                                    P_DRIVE_RIGHT_master_talFX = 2,
                                    P_DRIVE_LEFT_follow_talFX = 1,
                                    P_DRIVE_RIGHT_follow_talFX = 3;
        
        public static final double C_TRACK_WIDTH_METERS = 0.5207;
        public static final double C_DRIVE_EPR = 22016;
        public static final double C_WHEEL_DIAMETER_METERS = 0.1524;

        // Feedforward Constants
        public static final double C_kS = 0.133,
                                    C_kV = 2.48,
                                    C_kA = 0.101;

        // Left PID Constants
        public static double C_kP_LEFT = 4.52,
                                C_kI_LEFT = 0.00,
                                C_kD_LEFT = 0.00;

        // Right PID Constants
        public static double C_kP_RIGHT = 4.07,
                                C_kI_RIGHT = 0.00,
                                C_kD_RIGHT = 0.00;

        // Drive Distance PID Constants
        public static double C_kP_DIST = 0.9,
                                    C_kI_DIST = 0.00, 
                                    C_kD_DIST = 0.00;

        // Turn PID Constants

        public static double C_kP_turn = 13,
                                    C_kI_turn = 0.00, 
                                    C_kD_turn = 0.00;
        
        // Radians per second
        public static final double C_maxVel_turn = 1,
                                    C_maxAccel_turn = 1;

        // Trajectory Constants
        public static final double C_maxVel = 2,
                                    C_maxAccel = 2;

        public static final double C_kB_RAMSETE = 2.0,
                                    C_kZeta_RAMSETE = 0.7;
        
        public static double ticksToMeters(double ticks){
            return ticks*Math.PI*C_WHEEL_DIAMETER_METERS/C_DRIVE_EPR;
        }
    
        public static double radiansToMeters(double radians){
            return C_TRACK_WIDTH_METERS/2 * radians;
        }
    
        public static double metersToRadians(double meters){
            return meters * 2/C_TRACK_WIDTH_METERS;
        }
        
    }
    
    //--------------------------------------------------------------------------------------------------
    // Constants for subsystems involved with PowerCells (Balls)

    public static final class PowerCellConstants{         // TODO: Change port/name later*****************************
        // Hood Constants
        public static final int P_HOOD_sol = 0;

        // Intake Constants
        public static final int P_INTAKE_spMAX_1 = 4;           
        public static final int P_INTAKE_spMAX_2 = 5;
        
        public static final int P_INTAKE_sol_1 = 0;
        public static final int P_INTAKE_sol_2 = 0;

        public static final int C_INTAKE_SPEED = 1;

        // Shooter Constants
        public static final int P_SHOOTER_spMAX_1 = 4;
        public static final int P_SHOOTER_spMAX_2 = 5;
        
        public static final double C_kS = 0.0,
                                    C_kV = 0.0,
                                    C_kA = 0.0;
        
        public static final double C_kP = 0.0,
                                    C_kI = 0.0,
                                    C_kD = 0.0;

        public static final double C_SHOOTER_EPR = 42;

        public static final double C_SHOOTER_SPEED_CLOSE = 0.0,
                                    C_SHOOTER_SPEED_TRENCH = 0.0; // Velocity in rpm

        public static final double C_SHOOTER_SPEED_TOLERANCE = 0.0;
        
        public static enum E_SHOOT_POS{
            CLOSE,
            TRENCH;
        }
        
        // Magazine Constants
        public static final int P_MAGAZINE_spMAX_1 = 4;
        public static final int P_MAGAZINE_spMAX_2 = 5;
        
        public static final int C_MAGAZINE_SPEED = 1;

        public static final int[] P_MAGAZINE_limSwitch = {0, 1, 2, 3};

        // Pre-roller Constants
        public static final int P_PREROLLER_vicSPX = 0;     // TODO: Change to correct port number
        public static final double C_PREROLLER_SPEED = 0.5;
    }

    //--------------------------------------------------------------------------------------------------
    // Constants for subsystems involved with Lifting the Robot

    public static final class LiftConstants{
        
        // Elevator Constants
        public static final int P_ELEVATOR_motor_talSRX = 0;      // TODO: Change ports later******************************

        public static final double C_ELEVATOR_EPR = 1024*5;
        public static final double C_SPOOL_DIAMETER_METERS = 1;  
        
        public static double ticksToMeters(double ticks){
            return ticks*Math.PI*C_SPOOL_DIAMETER_METERS/C_ELEVATOR_EPR;
        }

        public static double talVelToMetersPerSec(double sensorVel){
            return ticksToMeters(sensorVel) * 10;
        }

        // Lift Constants
        public static final int P_LIFT_motor1_talSRX = 0;
        public static final int P_LIFT_motor2_talSRX = 0;
               
        // ************************************************
        // Command Constants
        public static final int C_eleMAX_SPEED = 1;

    }

    //--------------------------------------------------------------------------------------------------
    // Constants for JoySticks
    public static final class JoyConstants{
        public static final int P_OI_JOY_LEFT = 0,
                                P_OI_JOY_RIGHT = 1,
                                P_OI_JOY_MECH = 2;
    }
}