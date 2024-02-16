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
        public static final double AUTONOMOUS_MAX_ANGULAR_SPEED = 0.5;

        // max speed values in m/s
        public static final Supplier<Double> MAX_SPEED = new Supplier<Double>() {
            @Override
            public Double get() {
                return SmartDashboard.getNumber("max drive speed", 1);
            }
        };

        // speed values in deg/s
        public static final Supplier<Double> MAX_ANGULAR_SPEED = new Supplier<Double>() {
            @Override
            public Double get() {
                return SmartDashboard.getNumber("max angular speed", 180);
            }
        };

        public static final boolean USES_ABS_ENCODER = true;

        // chassis motor ports
        public static final int TOP_LEFT_DRIVE_PORT = 1;
        public static final int TOP_RIGHT_DRIVE_PORT = 6;
        public static final int DOWN_LEFT_DRIVE_PORT = 3;
        public static final int DOWN_RIGHT_DRIVE_PORT = 9;
        public static final int TOP_LEFT_STEERING_PORT = 62;
        public static final int TOP_RIGHT_STEERING_PORT = 7;
        public static final int DOWN_LEFT_STEERING_PORT = 2;
        public static final int DOWN_RIGHT_STEERING_PORT = 8;

        // chassis encoders
        public static final int TOP_LEFT_CANCODER = 4;
        public static final int TOP_RIGHT_CANCODER = 5;
        public static final int DOWN_LEFT_CANCODER = 0;
        public static final int DOWN_RIGHT_CANCODER = 1;

        //cancoder offsets
        public static final double TOP_RIGHT_CANCODER_OFFSET = -0.187500 - 2;
        public static final double TOP_LEFT_CANCODER_OFFSET = -1.966553;
        public static final double DOWN_RIGHT_CANCODER_OFFSET = -0.205078;
        public static final double DOWN_LEFT_CANCODER_OFFSET = -0.205078 -1.669434 ;

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
        public static final int INTAKE_MOTOR_ID = 19;

        public static final double INTAKE_SPEED = 0.7;

        public static final double INTAKE_MOTOR_KP = 0;
        public static final double INATKE_MOTOR_KI = 0;
        public static final double INATKE_MOTOR_KD = 0;
        public static final double INTAKE_MOTOR_FF = 0;   
    }

    public class ShooterValues{
        //speaker
        public static final Vector3d RED_SPAKER_POS = new Vector3d(652.67 * 0.0254, 2.1 , 218.42 * 0.0254);
        public static final Vector3d BLUE_SPAKER_POS = new Vector3d(-1.5 * 0.0254, 2.1 , 218.41 * 0.0254);

        //gear ratio
        public static final double AIM_MOTOR_GEAR_RATIO = 1 / 30.0;

        //shooter speeds(in rpm)
        public static final double SPEAKER_SHOOT_SPEED = 0;
        public static final double AMP_SHOOT_SPEED = 0;
        public static final double CONTAINMENT_SPEED = 0;
        
        //shooter angles
        public static final double AIM_MOTOR_MAX_ANGLE = 180;
        public static final double AIM_MOTOR_MIN_ANGLE = -42;
        public static final double AIM_MOTOR_AMP_ANGLE = 150;
        public static final double AIM_MOTOR_SPEAKER_ANGLE = 0;
        
        //motor controllers ids
        public static final int RIGHT_SHOOT_MOTOR_ID = 6;
        public static final int LEFT_SHOOT_MOTOR_ID = 1;
        public static final int CONTAINMENT_MOTOR_ID = 7;
        public static final int AIM_MOTOR_ID = 9;

        //limit switches ids
        public static final int TOP_LIMIT_SWITCH_ID = 0;
        public static final int BOTTOM_LIMIT_SWITCH_ID = 0;

        //color min confidence
        public static final double COLOR_SENSOR_CONFIDENCE = 0.5;

        //inverted motors
        public static final boolean RIGHT_SHOOT_MOTOR_INVERTED = false;
        public static final boolean LEFT_SHOOT_MOTOR_INVERTED = true;
        public static final boolean CONTAINMENT_MOTOR_INVERTED = false;
        public static final boolean AIM_MOTOR_INVERTED = false;
        
        //motors idle mode
        public static final IdleMode RIGHT_SHOOT_IDLE_MODE = IdleMode.kBrake;
        public static final IdleMode LEFT_SHOOT_IDLE_MODE = IdleMode.kBrake;
        public static final IdleMode CONTAINMENT_IDLE_MODE = IdleMode.kBrake;
        public static final IdleMode AIM_IDLE_MODE = IdleMode.kBrake;

        //distance to angle func values
        public static final double DISTANCE_TO_ANGLE_M = 0;
        public static final double DISTANCE_TO_ANGLE_B = 0;
    }    

    public class JoystickValues {
        public static final int CHASSIS = 0;
        public static final int OPERATOR = 1;
        public static final double JOYSTICK_DEADZONE = 0.2;
    }

    public class VisionValues{
        public static final int LOCALIZATION_VISION_PORT = 5800;
    }

}