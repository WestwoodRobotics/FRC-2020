package frc.robot;

//Create a new "constants" class for each mech
public final class Constants{
    
    
    //Constants for the DriveTrain
    //All units in meters and meters per second
    public static final class DriveConstants{
        
        // Motor Controller Ports
        public static final int P_DRIVE_LEFT_master_talFX = 0,
                                    P_DRIVE_RIGHT_master_talFX = 2,
                                    P_DRIVE_LEFT_follow_talFX = 1,
                                    P_DRIVE_RIGHT_follow_talFX = 3;
        
        public static final double C_TRACK_WIDTH_METERS = 0.5207;
        public static final double C_EPR = 22016;
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
        public static double C_kP_DIST = 0.00,
                                    C_kI_DIST = 0.00, 
                                    C_kD_DIST = 0.00;


        // Turn PID Constants
        public static double C_kP_turn = 0.00,
                                    C_kI_turn = 0.00, 
                                    C_kD_turn = 0.00;
        
        public static final double C_maxVel_turn = 12/C_kV,
                                    C_maxAccel_turn = 10;

        // Trajectory Constants
        public static final double C_maxVel = 0.5,
                                    C_maxAccel = 0.5;

        public static final double C_kB_RAMSETE = 2.0,
                                    C_kZeta_RAMSETE = 0.7;
        
    }

    public static final class IntakeConstants{
        public static final int P_SHOOTER_spMAX_1 = 4;
        public static final int P_SHOOTER_spMAX_2 = 5;
    }

    //Constants for JoySticks
    public static final class JoyConstants{
        public static final int P_OI_JOY_LEFT = 0,
                                P_OI_JOY_RIGHT = 1;
    }
}