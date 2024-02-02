package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.Consts;

public class Shooter extends SubsystemBase implements Consts{
    private static Shooter m_instance;
    //controls the right side shooter rollers
    private CANSparkMax m_rightShootMotor;
    //controls the left side shooter roller
    private CANSparkMax m_leftShootMotor;
    //controls the motor that puts note inside
    private CANSparkMax m_containmentMotor;
    //controls the angle of the shooter
    private CANSparkMax m_aimMotor;
    //sends signal when the shooter is at the top angle
    private DigitalInput m_topLimitSwitch;
    //sends signal when the shooter is at the bottom angle
    private DigitalInput m_bottomLimitSwitch;
    //detect when the note is inside to shooter 
    private ColorSensorV3 m_colorSensor;
    private ColorMatch m_colorMatcher;

    private Shooter(){
        //create motor controller objects
        m_rightShootMotor = new CANSparkMax(ShooterValues.RIGHT_SHOOT_MOTOR_ID, MotorType.kBrushless);
        m_leftShootMotor = new CANSparkMax(ShooterValues.LEFT_SHOOT_MOTOR_ID, MotorType.kBrushless);
        m_containmentMotor = new CANSparkMax(ShooterValues.CONTAINMENT_MOTOR_ID, MotorType.kBrushless);
        m_aimMotor = new CANSparkMax(ShooterValues.AIM_MOTOR_ID, MotorType.kBrushless);
        
        //create limit switches objects
        m_topLimitSwitch = new DigitalInput(ShooterValues.TOP_LIMIT_SWITCH_ID);
        m_bottomLimitSwitch = new DigitalInput(ShooterValues.BOTTOM_LIMIT_SWITCH_ID);

        //create color sensor object 
        m_colorSensor = new ColorSensorV3(I2C.Port.kOnboard);

        //create color matcher object 
        m_colorMatcher = new ColorMatch();
        m_colorMatcher.addColorMatch(Color.kOrange);
        m_colorMatcher.addColorMatch(Color.kOrangeRed);
        m_colorMatcher.addColorMatch(Color.kDarkOrange);

        //reset factory defaults
        m_rightShootMotor.restoreFactoryDefaults();
        m_leftShootMotor.restoreFactoryDefaults();
        m_containmentMotor.restoreFactoryDefaults();
        m_aimMotor.restoreFactoryDefaults();

        //set inverted
        m_rightShootMotor.setInverted(ShooterValues.RIGHT_SHOOT_MOTOR_INVERTED);
        m_leftShootMotor.setInverted(ShooterValues.LEFT_SHOOT_MOTOR_INVERTED);
        m_containmentMotor.setInverted(ShooterValues.CONTAINMENT_MOTOR_INVERTED);
        m_aimMotor.setInverted(ShooterValues.AIM_MOTOR_INVERTED);

        //set pid values
        m_rightShootMotor.getPIDController().setP(PIDValues.RIGHT_ROLLERS_SPEED_KP);
        m_rightShootMotor.getPIDController().setI(PIDValues.RIGHT_ROLLERS_SPEED_KI);
        m_rightShootMotor.getPIDController().setD(PIDValues.RIGHT_ROLLERS_SPEED_KD);
        m_rightShootMotor.getPIDController().setFF(PIDValues.RIGHT_ROLLERS_SPEED_KF);
        
        m_leftShootMotor.getPIDController().setP(PIDValues.LEFT_ROLLERS_SPEED_KP);
        m_leftShootMotor.getPIDController().setI(PIDValues.LEFT_ROLLERS_SPEED_KI);
        m_leftShootMotor.getPIDController().setD(PIDValues.LEFT_ROLLERS_SPEED_KD);
        m_leftShootMotor.getPIDController().setFF(PIDValues.LEFT_ROLLERS_SPEED_KF);

        m_containmentMotor.getPIDController().setP(PIDValues.CONTAINMENT_SPEED_KP);
        m_containmentMotor.getPIDController().setI(PIDValues.CONTAINMENT_SPEED_KI);
        m_containmentMotor.getPIDController().setD(PIDValues.CONTAINMENT_SPEED_KD);
        m_containmentMotor.getPIDController().setFF(PIDValues.CONTAINMENT_SPEED_KF);

        m_aimMotor.getPIDController().setP(PIDValues.SHOOTER_ANGLE_KP);
        m_aimMotor.getPIDController().setI(PIDValues.SHOOTER_ANGLE_KI);
        m_aimMotor.getPIDController().setD(PIDValues.SHOOTER_ANGLE_KD);

        //set idle mode
        m_rightShootMotor.setIdleMode(IdleMode.kBrake);
        m_leftShootMotor.setIdleMode(IdleMode.kBrake);

        //gear ratio
        m_aimMotor.getEncoder().setPositionConversionFactor(ShooterValues.AIM_MOTOR_GEAR_RATIO * 360); //convert to degrees
    }

    /**
     * @return only instance of this class
     */
    public static Shooter getInstance(){
        if(m_instance == null)
            m_instance = new Shooter();
        return m_instance;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("shooter angle", getShooterAngle());
        SmartDashboard.putNumber("right rollers speed", getRightRollersSpeed());
        SmartDashboard.putNumber("left rollers speed", getLeftRollerSpeed());
        
        //set encoder position value when touches limit switches
        if(m_topLimitSwitch.get())
            m_aimMotor.getEncoder().setPosition(ShooterValues.AIM_MOTOR_MAX_ANGLE);
        if(m_bottomLimitSwitch.get())
            m_aimMotor.getEncoder().setPosition(ShooterValues.AIM_MOTOR_MIN_ANGLE);    

        //start spining rollers if note is inside the shooter 
        ColorMatchResult result = m_colorMatcher.matchColor(m_colorSensor.getColor());
        if(result != null && result.confidence >= ShooterValues.COLOR_SENSOR_CONFIDENCE)
            shootNote(getLeftRollerSpeed());

        
    }

    /**
     * Turn the shooter to given angle
     * @param angle - target shooter angle in degrees
     */
    public void turnToAngle(double angle){
        //clamp value to make sure bad input wont break the robot
        angle = MathUtil.clamp(angle, ShooterValues.AIM_MOTOR_MIN_ANGLE, ShooterValues.AIM_MOTOR_MAX_ANGLE);
        //scale kf according to the shooter's angle(watch the sinus function graph inorder to understand why its here)
        m_aimMotor.getPIDController().setFF(Math.sin(Math.toRadians(angle)) * PIDValues.SHOOTER_ANGLE_KF);
        //activate pid
        m_aimMotor.getPIDController().setReference(angle, ControlType.kPosition);
    }

    /**
     * 
     * @return current shooter angle
     */
    public double getShooterAngle(){
        return m_aimMotor.getEncoder().getPosition();
    }

    /**
     * Shoot note
     * @param speed - target shoot speed in rpm
     */
    public void shootNote(double speed){
        //activate velocity pid on right rollers
        m_rightShootMotor.getPIDController().setReference(speed, ControlType.kVelocity);
        //activate velocity pid on left roller
        m_leftShootMotor.getPIDController().setReference(speed, ControlType.kVelocity);
    }

    /**
     * Push the note to the shooter rollers 
     * @param speed - target speed in rpm
     */
    public void pushNoteToRollers(double speed){
        m_containmentMotor.getPIDController().setReference(speed, ControlType.kVelocity);
    }

    /**
     * 
     * @return current speed of the right rollers
     */
    public double getRightRollersSpeed(){
        return m_rightShootMotor.getEncoder().getVelocity();
    }

    /**
     * 
     * @return current speed of the left roller
     */
    public double getLeftRollerSpeed(){
        return m_leftShootMotor.getEncoder().getVelocity();
    }

    /**
     * Pull the note into shooter 
     * @param speed - containment speed in rpm
     */
    public void pullNote(double speed){
        //activate velocity pid on right rollers motor controller
        m_rightShootMotor.getPIDController().setReference(speed, ControlType.kVelocity);
        //activate velocity pid on left rollers motor controller
        m_leftShootMotor.getPIDController().setReference(speed, ControlType.kVelocity);
        //activate velocity pid on containment roller motor controller
        m_containmentMotor.getPIDController().setReference(speed, ControlType.kVelocity);
    }

    /**
     * stop spining rollers 
     */
    public void stopRollers(){
        m_leftShootMotor.set(0);
        m_rightShootMotor.set(0);
        m_containmentMotor.set(0);
    }


}
