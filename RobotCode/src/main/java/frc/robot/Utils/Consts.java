package frc.robot.Utils;

import java.util.function.Supplier;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public interface Consts {

    public class PIDValues{
        //wheel angle pid
        public static final double WHEEL_ANGLE_KP = 0.01;
        public static final double WHEEL_ANGLE_KI = 0.0;
        public static final double WHEEL_ANGLE_KD = 0.0;

        //wheel velocity pid
        public static final double WHEEL_VELOCITY_KP = 0.1;
        public static final double WHEEL_VELOCITY_KI = 0;
        public static final double WHEEL_VELOCITY_KD = 0;
        public static final double WHEEL_VELOCITY_KF = 0.75 / 2.81;

        //pos pid
        public static final double POS_KP = 10;
        public static final double POS_KI = 0;
        public static final double POS_KD = 0;

        public static final double X_TOLERANCE = 0.12;
        public static final double Y_TOLERANCE = 0.12;

        //speed values
        public static final double HEADING_KP = 0.02675;
        public static final double HEADING_KI = 0;
        public static final double HEADING_KD = 0;
        public static final double HEADING_TOLERANCE = 2;
        
        //shooter angle
        public static final double SHOOTER_ANGLE_KP = 0.0111;//0.011
        public static final double SHOOTER_ANGLE_KI = 0.000001901; //0.0000003
        public static final double SHOOTER_ANGLE_KD = 0.00015;//0.03
        public static final double SHOOTER_ANGLE_KF = 0;//0.054 / 2;//0.057 / 2
        

        //right rollers speed
        public static final double RIGHT_ROLLERS_SPEED_KP = 0.00009;// 0.0006 * 0.9;
        public static final double RIGHT_ROLLERS_SPEED_KI = 0.00000074;// 0.0000003 * 0.9;
        public static final double RIGHT_ROLLERS_SPEED_KD = 0.001;// 0.00001;
        public static final double RIGHT_ROLLERS_SPEED_KF = 0;// 0.00001;
        
        //left rollers speed
        public static final double LEFT_ROLLERS_SPEED_KP = 0.00009;//0.0003 * 0.9;
        public static final double LEFT_ROLLERS_SPEED_KI = 0.00000074;//0.0000006 * 0.9;
        public static final double LEFT_ROLLERS_SPEED_KD = 0.001;//0.000001;
        public static final double LEFT_ROLLERS_SPEED_KF = 0;//0.000002;
        
        //containment speed
        public static final double CONTAINMENT_SPEED_KP = 0.00009;
        public static final double CONTAINMENT_SPEED_KI = 0.00000069;
        public static final double CONTAINMENT_SPEED_KD = 0.001;
        public static final double CONTAINMENT_SPEED_KF = 0.000;
        
    }

    public class ChassisValues{
        public static double DRIVE_SPEED = 2.8; //m/sec
        public static double ANGULAR_SPEED = 200; //deg/sec
        public static double SLOW_MODE_DRIVE_SPEED = 1;
        public static double FAST_MODE_DRIVE_SPEED = 4.5;
        public static double AUTO_DRIVE_SPEED = 1.5;
        public static double MIN_DIFF_FOR_ANGLE_OFFSET_REPLACEMENT = 10.0; // minimum difference between the new calculated angle offset and the old angle offset to replace the old offset by the new offset (degrees)
        public static double ANGLE_OFFSET_AVERAGE_NEW_READING_WEIGHT = 0.5; // how much weight do we give the new offset when averaging with the last offset
        // (theoretically) the lower the number the less offset drifting
        // keep in mind this number should be between 0 and 1 with 0 meaning that we'll always stick to the old offset and 1 meaning we'll always replace the offset not caring about the old offset
        // also keep in mind that even if you set it to 0 the offset will still update if the new offset is drastically different than the last

        // max speed values in m/s
        public static Supplier<Double> MAX_SPEED = new Supplier<Double>() {
            @Override
            public Double get() {
                return SmartDashboard.getNumber("max drive speed", DRIVE_SPEED);
            }
        };

        // speed values in deg/s
        public static final Supplier<Double> MAX_ANGULAR_SPEED = new Supplier<Double>() {
            @Override
            public Double get() {
                return SmartDashboard.getNumber("max angular speed", 200);
            }
        };

        public static final boolean USES_ABS_ENCODER = true;

        // chassis motor ports
        public static final int TOP_LEFT_DRIVE_PORT = 18;
        public static final int TOP_RIGHT_DRIVE_PORT = 17;
        public static final int DOWN_LEFT_DRIVE_PORT = 3;
        public static final int DOWN_RIGHT_DRIVE_PORT = 27;
        public static final int TOP_LEFT_STEERING_PORT = 62;
        public static final int TOP_RIGHT_STEERING_PORT = 13;
        public static final int DOWN_LEFT_STEERING_PORT = 2;
        public static final int DOWN_RIGHT_STEERING_PORT = 8;

        // chassis encoders
        public static final int TOP_LEFT_CANCODER = 4;
        public static final int TOP_RIGHT_CANCODER = 5;
        public static final int DOWN_LEFT_CANCODER = 0;
        public static final int DOWN_RIGHT_CANCODER = 1;

        //cancoder offsets
        public static final double TOP_RIGHT_CANCODER_OFFSET = -2.187255859375;
        public static final double TOP_LEFT_CANCODER_OFFSET = -1.64965820312;
        public static final double DOWN_RIGHT_CANCODER_OFFSET = -1.2109375;
        public static final double DOWN_LEFT_CANCODER_OFFSET = -0.87744140625;

        // chassis size
        public static final double FRONT_WHEEL_DIST_METERS = 0.57;
        public static final double SIDE_WHEEL_DIST_METERS = 0.57;
        public static final double WHEEL_PERIMETER = Math.PI * 0.095;
        public static final double ROBOT_BOUNDING_CIRCLE_PERIMETER = Math.PI * Math.sqrt(
            FRONT_WHEEL_DIST_METERS * FRONT_WHEEL_DIST_METERS + SIDE_WHEEL_DIST_METERS * SIDE_WHEEL_DIST_METERS);

        // swerve vectors
        public static final Vector2d TOP_RIGHT = new Vector2d((FRONT_WHEEL_DIST_METERS / 2),
                (SIDE_WHEEL_DIST_METERS / 2));
        public static final Vector2d TOP_LEFT = new Vector2d(-(FRONT_WHEEL_DIST_METERS / 2),
                SIDE_WHEEL_DIST_METERS / 2);
        public static final Vector2d DOWN_RIGHT = new Vector2d(FRONT_WHEEL_DIST_METERS / 2,
                -(SIDE_WHEEL_DIST_METERS / 2));
        public static final Vector2d DOWN_LEFT = new Vector2d(-(FRONT_WHEEL_DIST_METERS / 2),
                -(SIDE_WHEEL_DIST_METERS / 2));

        // array of physical module vectors
        public static final Vector2d[] physicalMoudulesVector = { TOP_RIGHT, TOP_LEFT,
                DOWN_RIGHT, DOWN_LEFT };// array of vectors from robot center to swerves module

        // module gear ratios
        public static final double DRIVE_GEAR_RATIO = 1 / 6.75; // L2
        public static final double STEERING_GEAR_RATIO = 1 / 12.8;


        public static final double NEXT_POS_ESTIMATION_DELTA_TIME = 0.3;
    }

    public class ClimberValues{
        public static final int CLIMBER_MOTOR_RIGHT_PORT = 15;
        public static final int CLIMBER_MOTOR_LEFT_PORT = 61;

        public static final int CLIMBER_LIMIT_SWITCH_RIGHT = 1;
        public static final int CLIMBER_LIMIT_SWITCH_LEFT = 2;

        public static final double CLIMBER_SPEED = 1;
        public static final double CLIMBER_RPM = 0;

        public static final int CLIMBER_LIMIT_SWITCH_RIGHT_PORT = 0;
        public static final int CLIMBER_LIMIT_SWITCH_LEFT_PORT = 0;

        public static final double CLIMBER_GEAR_RATIO = 1.0 / 45.0;

        public static final double CLIMBER_KP = 0;
        public static final double CLIMBER_KI = 0;
        public static final double CLIMBER_KD = 0;
        public static final double CLIMBER_FF = 0;

        public static final boolean IS_MOTOR_RIGHT_INVERTED = true;       
        public static final boolean IS_MOTOR_LEFT_INVERTED = false;
        

        public static final double CLIMBER_FORCE_STOP_TOLERANCE = 0.01;
    }

    public class IntakeValues{
        public static final int INTAKE_MOTOR_ID = 19;

        public static final double INTAKE_SPEED = 0.7;

        public static final int INTAKE_CURRENT_LIMIT = 60;
        public static final double INTAKE_MOTOR_KP = 0;
        public static final double INATKE_MOTOR_KI = 0;
        public static final double INATKE_MOTOR_KD = 0;
        public static final double INTAKE_MOTOR_FF = 0;   
    }

    public class ShooterValues{
        //speaker
        public static final Vector3d RED_SPAKER_POS = new Vector3d(652.67 * 0.0254 + 0.05, 2.05, 218.42 * 0.0254 - 0.4);
        public static final Vector3d BLUE_SPAKER_POS = new Vector3d(-1.5 * 0.0254 - 0.05, 2.05, 218.41 * 0.0254 + 0.4);
        public static final double SPEAKER_HEIGHT_SCALAR = 0.12216; 

        //gear ratio
        public static final double AIM_MOTOR_GEAR_RATIO = 1 / (45.0);

        //shooter speeds(in rpm)
        public static final double SPEAKER_SHOOT_SPEED = 6000;
        public static final double AMP_SHOOT_SPEED = 400;//1700
        public static final double CONTAINMENT_SPEED = 6000;
        public static final double SHOOT_SPEED_TOLERANCE = 500;
        // public static final double SHOOTER_AIM_MAX_VELOCITY = 260.0; // max RPM of shooter aim motor where its okay to shoot
        
        //shooter angles
        public static final double AIM_MOTOR_MAX_ANGLE = 180;
        public static final double AIM_MOTOR_MIN_ANGLE = -50.4;
        public static final double AIM_MOTOR_AMP_ANGLE = 98;//130 works on our amp not comp's
        public static final double AIM_MOTOR_SPEAKER_ANGLE = 0;
        public static final int AIM_MOTOR_CURRENT_LIMIT = 20;
        public static final double AIM_MOTOR_RATE_LIMIT = 1/9.0;
        public static final double AIM_MOTOR_SPEED_LIMIT = 0.2;
        public static final double AIM_MOTOR_MIN_SPEED = 0.0001;
        public static final double AIM_MOTOR_MIN_TOLERANCE = 3.0;

        //motor controllers ids
        public static final int RIGHT_SHOOT_MOTOR_ID = 6;
        public static final int LEFT_SHOOT_MOTOR_ID = 1;
        public static final int CONTAINMENT_MOTOR_ID = 7;
        public static final int AIM_MOTOR_ID = 9;
        public static final int NOTE_SENSOR_PORT = 3;

        //limit switches ids
        public static final int TOP_LIMIT_SWITCH_ID = 0;
        public static final int BOTTOM_LIMIT_SWITCH_ID = 9;

        //encoder
        public static final int EXTERNAL_AIM_MOTOR_ENCODER_ID = 0;//connected to the DIO

        //inverted motors
        public static final boolean RIGHT_SHOOT_MOTOR_INVERTED = false;
        public static final boolean LEFT_SHOOT_MOTOR_INVERTED = true;
        public static final boolean CONTAINMENT_MOTOR_INVERTED = false;
        public static final boolean AIM_MOTOR_INVERTED = true;
        
        //motors idle mode
        public static final IdleMode RIGHT_SHOOT_IDLE_MODE = IdleMode.kCoast;
        public static final IdleMode LEFT_SHOOT_IDLE_MODE = IdleMode.kCoast;
        public static final IdleMode CONTAINMENT_IDLE_MODE = IdleMode.kBrake;
        public static final IdleMode AIM_IDLE_MODE = IdleMode.kBrake;

        // physical constants
        public static final double SHOOTER_HEIGHT_METERS = 0.38287; // distance between the shooter axis of rotation to the ground

        public static final double EXTERNAL_ENCODER_OFFSET = 0.222401380560035;

        public static final double MIN_NOTE_DISTANCE = 20.0;//20.0

        
    }    

    public class JoystickValues {
        public static final int CHASSIS = 0;
        public static final int OPERATOR = 1;
        public static final double JOYSTICK_DEADZONE = 0.2;
    }

    public class VisionValues{
        public static final int LOCALIZATION_VISION_PORT = 5800;
        public static final int JETSON_HEALTH_CHECK_PORT = 5801;
        public static final double VISION_FRAME_TIME = 1.0 / 20.0;
    }


    public class PhysicalConsts {
        public static final double FIELD_HEIGHT = 8.21055;
        public static final double FIELD_WIDTH = 16.54175;
    
        
    }
}