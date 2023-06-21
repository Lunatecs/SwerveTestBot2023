// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class Constants {
    public final static class DrivetrainConstants {
        public static final int LEFT_FRONT_DRIVE = 9;//TODO: Update CAN values of drive & turning motors
        public static final int LEFT_FRONT_TURN = 8;
        public static final int LEFT_FRONT_ENC = 2;

        public static final int RIGHT_FRONT_DRIVE = 10;
        public static final int RIGHT_FRONT_TURN = 13;
        public static final int RIGHT_FRONT_ENC = 4;

        public static final int LEFT_BACK_DRIVE = 7;
        public static final int LEFT_BACK_TURN = 11;
        public static final int LEFT_BACK_ENC = 5;

        public static final int RIGHT_BACK_DRIVE = 6;
        public static final int RIGHT_BACK_TURN = 12;
        public static final int RIGHT_BACK_ENC = 3;

        public static final int PIGEON = 1;
    }

    public final static class JoystickConstants{
    
        public final static int DRIVER_USB = 0;
        public final static int OPERATOR_USB = 1;
        public final static int TEST_USB = 2;
        
        public final static int LEFT_Y_AXIS = 1;
        public final static int LEFT_X_AXIS = 0;
        public final static int RIGHT_X_AXIS = 4;
    
    
        public final static int GREEN_BUTTON = 1;
        public final static int RED_BUTTON = 2;
        public final static int YELLOW_BUTTON = 4;
        public final static int BLUE_BUTTON = 3;
    
        public final static int LEFT_TRIGGER = 2;
        public final static int RIGHT_TRIGGER = 3;
        public final static int LEFT_BUMPER = 5;
        public final static int RIGHT_BUMPER = 6;
    
        public final static int BACK_BUTTON = 7;
        public final static int START_BUTTON = 8;
    
        public final static int POV_UP = 0;
        public final static int POV_RIGHT = 90;
        public final static int POV_DOWN = 180;
        public final static int POV_LEFT = 270;
      }

      public final static class SwerveConstants {
        public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.2794;
        public static final double DRIVETRAIN_WHEELBASE_METERS = 0.2794;

        public static final int DRIVETRAIN_PIGEON_ID = 0;

        public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 3;
        public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 4;
        public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 2;
        public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(238.97);
        public static final double FRONT_LEFT_OFFSET = 0.0;
        public static final boolean FL_REVERSED = false;

        public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 8;
        public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 7;
        public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 4;
        public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(140.62);
        public static final double FRONT_RIGHT_OFFSET = 0.0;
        public static final boolean FR_REVERSED = false;

        public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 1;
        public static final int BACK_LEFT_MODULE_STEER_MOTOR = 2;
        public static final int BACK_LEFT_MODULE_STEER_ENCODER = 3;
        public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(325.63);
        public static final double BACK_LEFT_OFFSET = 0.0;
        public static final boolean BL_REVERSED = false;

        public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 6;
        public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 5;
        public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 1;
        public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(266.48);
        public static final double BACK_RIGHT_OFFSET = 0.0;
        public static final boolean BR_REVERSED = false;
      }
}
