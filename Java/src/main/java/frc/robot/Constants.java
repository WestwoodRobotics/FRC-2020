package frc.robot;

//Create a new "constants" class for each mech
public final class Constants{
    
    
    //Constants for the DriveTrain
    public static final class DriveConstants{
        
        // Motor Controller Ports
        public static final int P_DRIVE_LEFT_master_vicSPX = 0, //Change to 0 after done testing Falcons
                                        P_DRIVE_RIGHT_master_vicSPX = 2, //Change to 2 after done testing Falcons
                                        P_DRIVE_LEFT_follow_vicSPX = 1,
                                        P_DRIVE_RIGHT_follow_vicSPX = 3;
        

        // Turn PID Constants
        public static double C_kP_turn = 17.8,
                                    C_kI_turn = 0.0, 
                                    C_kD_turn = 6.76,
                                    C_kS_turn = 0.274,
                                    C_kV_turn = 2.53,
                                    C_kA_turn = 0.165;

        public static double C_maxVel_turn = 4.743,
                                    C_maxAccel_turn = 10;
    }




    public static final class IntakeConstants{
        public static final int P_SHOOTER_vicSPX_1 = 4;
        public static final int P_SHOOTER_vicSPX_2 = 5;
    }

    //Constants for JoySticks
    public static final class JoyConstants{
        public static final int P_OI_JOY_LEFT = 0,
                                        P_OI_JOY_RIGHT = 2;
    }
}