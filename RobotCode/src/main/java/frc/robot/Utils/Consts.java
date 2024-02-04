package frc.robot.Utils;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public interface Consts {

    public class PIDValues{
        //wheel angle pid
        public static final double WHEEL_ANGLE_KP = 0.01;
        public static final double WHEEL_ANGLE_KI = 0.0;
        public static final double WHEEL_ANGLE_KD = 0.0;

        //wheel velocity pid
        public static final double WHEEL_VELOCITY_KP = 0.05;
        public static final double WHEEL_VELOCITY_KI = 0;
        public static final double WHEEL_VELOCITY_KD = 0;
        public static final double WHEEL_VELOCITY_KF = 0.75 / 2.81;

        //pos pid
        public static final double POS_KP = 15;
        public static final double POS_KI = 0;//5
        public static final double POS_KD = 0;//2

        public static final double X_TOLERANCE = 0.05;
        public static final double Y_TOLERANCE = 0.05;

        //speed values
        public static final double HEADING_KP = 0.03;
        public static final double HEADING_KI = 0;
        public static final double HEADING_KD = 0;
        public static final double HEADING_TOLERANCE = 3;
        
        //shooter angle
        public static final double SHOOTER_ANGLE_KP = 0;
        public static final double SHOOTER_ANGLE_KI = 0;
        public static final double SHOOTER_ANGLE_KD = 0;
        public static final double SHOOTER_ANGLE_KF = 0;
        
        //right rollers speed
        public static final double RIGHT_ROLLERS_SPEED_KP = 0;
        public static final double RIGHT_ROLLERS_SPEED_KI = 0;
        public static final double RIGHT_ROLLERS_SPEED_KD = 0;
        public static final double RIGHT_ROLLERS_SPEED_KF = 0;
        
        //left rollers speed
        public static final double LEFT_ROLLERS_SPEED_KP = 0;
        public static final double LEFT_ROLLERS_SPEED_KI = 0;
        public static final double LEFT_ROLLERS_SPEED_KD = 0;
        public static final double LEFT_ROLLERS_SPEED_KF = 0;
        
        //containment speed
        public static final double CONTAINMENT_SPEED_KP = 0;
        public static final double CONTAINMENT_SPEED_KI = 0;
        public static final double CONTAINMENT_SPEED_KD = 0;
        public static final double CONTAINMENT_SPEED_KF = 0;
        
    }

    public class ChassisValues{
        public static final boolean USES_ABS_ENCODER = true;

        // chassis motor ports
        public static final int TOP_LEFT_DRIVE_PORT = 6;
        public static final int TOP_RIGHT_DRIVE_PORT = 8;
        public static final int DOWN_LEFT_DRIVE_PORT = 4;
        public static final int DOWN_RIGHT_DRIVE_PORT = 10;
        public static final int TOP_LEFT_STEERING_PORT = 7;
        public static final int TOP_RIGHT_STEERING_PORT = 9;
        public static final int DOWN_LEFT_STEERING_PORT = 5;
        public static final int DOWN_RIGHT_STEERING_PORT = 11;

        // chassis encoders
        public static final int TOP_LEFT_CANCODER = 1;
        public static final int TOP_RIGHT_CANCODER = 4;
        public static final int DOWN_LEFT_CANCODER = 5;
        public static final int DOWN_RIGHT_CANCODER = 0;

        //cancoder offsets
        public static final double TOP_RIGHT_CANCODER_OFFSET = 0.293212890625;
        public static final double TOP_LEFT_CANCODER_OFFSET = 0.291748046875;
        public static final double DOWN_RIGHT_CANCODER_OFFSET = -0.1171875;
        public static final double DOWN_LEFT_CANCODER_OFFSET = 0.310546875;

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

    }

    public class ClimberValues{

    }

    public class IntakeValues{

    }

    public class ShooterValues{
        //gear ratio
        public static final double AIM_MOTOR_GEAR_RATIO = 1 / 30.0;

        //shooter speeds(in rpm)
        public static final double SPEAKER_SHOOT_SPEED = 0;
        public static final double AMP_SHOOT_SPEED = 0;
        public static final double CONTAINMENT_SPEED = 0;
        
        //shooter angle range
        public static final double AIM_MOTOR_MAX_ANGLE = 0;
        public static final double AIM_MOTOR_MIN_ANGLE = -42;
        
        //motor controllers ids
        public static final int RIGHT_SHOOT_MOTOR_ID = 0;
        public static final int LEFT_SHOOT_MOTOR_ID = 0;
        public static final int CONTAINMENT_MOTOR_ID = 0;
        public static final int AIM_MOTOR_ID = 0;

        //limit switches ids
        public static final int TOP_LIMIT_SWITCH_ID = 0;
        public static final int BOTTOM_LIMIT_SWITCH_ID = 0;

        //color min confidence
        public static final double COLOR_SENSOR_CONFIDENCE = 0.5;
        //inverted motors
        public static final boolean RIGHT_SHOOT_MOTOR_INVERTED = false;
        public static final boolean LEFT_SHOOT_MOTOR_INVERTED = false;
        public static final boolean CONTAINMENT_MOTOR_INVERTED = false;
        public static final boolean AIM_MOTOR_INVERTED = false;    
    }    

    public class JoystickValues {
        public static final int CHASSIS = 0;
        public static final int OPERATOR = 1;
        public static final double JOYSTICK_DEADZONE = 0.2;
    }

}