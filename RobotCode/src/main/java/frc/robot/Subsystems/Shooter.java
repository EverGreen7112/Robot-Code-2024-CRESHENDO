package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Utils.Consts;
import frc.robot.Utils.Vector2d;
import frc.robot.Utils.Vector3d;

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
    //external aim motor encoder
    public DutyCycleEncoder m_aimEncoder;
    //aim motor pid controller
    private PIDController m_aimPidController;
    //sends signal when the shooter is at the bottom angle
    private DigitalInput m_bottomLimitSwitch;
    //target angle
    private double m_targetAngle;
    //pid controller ff
    private double m_aimFF;
    //target shoot speed
    private double m_targetShootSpeed;
    // sensor to determine if note is in
    private AnalogInput m_noteSensor;


    private Shooter(){
        //create motor controller objects
        m_rightShootMotor = new CANSparkMax(ShooterValues.RIGHT_SHOOT_MOTOR_ID, MotorType.kBrushless);
        m_leftShootMotor = new CANSparkMax(ShooterValues.LEFT_SHOOT_MOTOR_ID, MotorType.kBrushless);
        m_containmentMotor = new CANSparkMax(ShooterValues.CONTAINMENT_MOTOR_ID, MotorType.kBrushless);
        m_aimMotor = new CANSparkMax(ShooterValues.AIM_MOTOR_ID, MotorType.kBrushless);
        
        //reset factory defaults
        m_rightShootMotor.restoreFactoryDefaults();
        m_leftShootMotor.restoreFactoryDefaults();
        m_containmentMotor.restoreFactoryDefaults();
        m_aimMotor.restoreFactoryDefaults();
        
        m_aimMotor.setSmartCurrentLimit(ShooterValues.AIM_MOTOR_CURRENT_LIMIT);
        m_aimMotor.setOpenLoopRampRate(ShooterValues.AIM_MOTOR_RATE_LIMIT);
        
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

        //set idle mode
        m_rightShootMotor.setIdleMode(ShooterValues.RIGHT_SHOOT_IDLE_MODE);
        m_leftShootMotor.setIdleMode(ShooterValues.LEFT_SHOOT_IDLE_MODE);
        m_aimMotor.setIdleMode(ShooterValues.AIM_IDLE_MODE);
        
        //create external encoder instance
        m_aimEncoder = new DutyCycleEncoder(ShooterValues.EXTERNAL_AIM_MOTOR_ENCODER_ID);
        m_aimEncoder.setPositionOffset(ShooterValues.EXTERNAL_ENCODER_OFFSET);
        m_aimEncoder.setDistancePerRotation(-360);
        
        m_targetAngle = ShooterValues.AIM_MOTOR_MIN_ANGLE;
        m_targetShootSpeed = 0;
        //create limit switch
        m_bottomLimitSwitch = new DigitalInput(ShooterValues.BOTTOM_LIMIT_SWITCH_ID);
        //create pid controller
        m_aimPidController = new PIDController(PIDValues.SHOOTER_ANGLE_KP, PIDValues.SHOOTER_ANGLE_KI, PIDValues.SHOOTER_ANGLE_KD);
        m_noteSensor = new AnalogInput(ShooterValues.NOTE_SENSOR_PORT);
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
        SmartDashboard.putNumber("note sensor value", (Math.pow(m_noteSensor.getAverageVoltage(), -1.2045) * 27.726));
        SmartDashboard.putNumber("shooter angle", getShooterAngle());
        SmartDashboard.putNumber("right rollers speed", getRightRollersSpeed());
        SmartDashboard.putNumber("left rollers speed", getLeftRollerSpeed());
        SmartDashboard.putBoolean("is ready to shoot", isReadyToShoot());
        SmartDashboard.putBoolean("is note in", isNoteIn());
        
        //set encoder position value when touches limit switches
        // if(!m_bottomLimitSwitch.get())
        //     m_aimEncoder.setPositionOffset((ShooterValues.AIM_MOTOR_MIN_ANGLE - m_aimEncoder.getDistance()) / (-360));
            // m_aimMotor.getEncoder().setPosition(ShooterValues.AIM_MOTOR_MIN_ANGLE); 
              
        
        
        m_aimPidController.setSetpoint(m_targetAngle);
        //calculate current output
        double output = MathUtil.clamp(m_aimPidController.calculate(m_aimEncoder.getDistance()), -0.3, 0.3);//put this in consts i dont have time
        //activate motor with ff
        m_aimMotor.set(output + m_aimFF * Math.signum(output));
    }

    /**
     * calculate the angle the shooter needs to be at according to the current distance from the speaker
     * @return the angle the shooter needs to be at
     */
    public double getShooterAngleToSpeaker(){
       
        Vector2d currentPos2d = Swerve.getInstance(ChassisValues.USES_ABS_ENCODER).getPos();
        //get position of shooter (NOT ROBOT)
        Vector3d currentPos = new Vector3d(currentPos2d.x, ShooterValues.SHOOTER_HEIGHT_METERS, currentPos2d.y);
        Vector3d speakerPos = new Vector3d();
       
        //get position of speaker according to the alliance
        if(Robot.getAlliance() == Alliance.Blue){
            speakerPos = ShooterValues.BLUE_SPAKER_POS;
        }
        else if(Robot.getAlliance() == Alliance.Red){
            speakerPos = ShooterValues.RED_SPAKER_POS;
        }

        //calculate the vector between them
        Vector3d delta = currentPos.subtract(speakerPos);
        return -Math.toDegrees(delta.getPitch());
    }


    /**
     * Turn the shooter to given angle
     * @param angle - target shooter angle in degrees
     */
    public void turnToAngle(double angle){
        //clamp value to make sure bad input won't break the robot
        angle = MathUtil.clamp(angle, ShooterValues.AIM_MOTOR_MIN_ANGLE, ShooterValues.AIM_MOTOR_MAX_ANGLE);
        //reset accumalted I after changing target angles direction
        if(Math.signum(angle - getShooterAngle()) != Math.signum(m_targetAngle - getShooterAngle()))
            m_aimPidController.reset();

        m_targetAngle = angle;
        //scale kf according to the shooter's angle(watch the cosinus function graph inorder to understand why its here)
        m_aimFF = Math.cos(Math.toRadians(m_targetAngle)) * PIDValues.SHOOTER_ANGLE_KF;
        //set target angle
        m_aimPidController.setSetpoint(m_targetAngle);
    }

    public void moveShooter(double speed){
        m_aimMotor.set(speed);
    }

    public double getTargetAngle(){
        return m_targetAngle;
    }

    /**
     * @return current shooter angle
     */
    public double getShooterAngle(){
        return m_aimEncoder.getDistance(); 
    }

    /**
     * Shoot note
     * @param speed - target shoot speed in rpm
     */
    public void setShootSpeed(double speed){
        m_targetShootSpeed = speed;
        //activate velocity pid on right rollers
        m_rightShootMotor.getPIDController().setReference(speed, ControlType.kVelocity);
        //activate velocity pid on left roller
        m_leftShootMotor.getPIDController().setReference(speed, ControlType.kVelocity);
    }

    /**
     * Shoot note
     * @param lSpeed - target shoot speed in rpm of left rollers
     * @param rSpeed - target shoot speed in rpm of right rollers
     */
    public void setShootSpeed(double lSpeed, double rSpeed){
        m_targetShootSpeed = (lSpeed > rSpeed) ? lSpeed: rSpeed;
        //activate velocity pid on right rollers
        m_leftShootMotor.getPIDController().setReference(lSpeed, ControlType.kVelocity);
        //activate velocity pid on left roller
        m_rightShootMotor.getPIDController().setReference(rSpeed, ControlType.kVelocity);
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
     * 
     * @return current speed of the containment roller
     */
    public double getContainmentSpeed(){
        return m_containmentMotor.getEncoder().getVelocity();
    }

    /**
     * Pull the note into shooter 
     * @param speed - containment speed in rpm
     */
    public void pullNote(double speed){
        m_targetShootSpeed = speed;
        //activate velocity pid on right rollers motor controller
        m_rightShootMotor.getPIDController().setReference(speed, ControlType.kVelocity);
        //activate velocity pid on left rollers motor controller
        m_leftShootMotor.getPIDController().setReference(speed, ControlType.kVelocity);
        //activate velocity pid on containment roller motor controller
        m_containmentMotor.getPIDController().setReference(speed, ControlType.kVelocity);
    }

    /**
     * Pull the note into shooter 
     * @param speed - containment speed in precentage
     */
    public void pullNoteWithoutPID(double speed){
        m_rightShootMotor.set(speed);
        m_leftShootMotor.set(speed);
        m_containmentMotor.set(speed);
    }

    /**
     * function used ONLY for testing dont use it.
     * @param speed
     */
    public void testMotors(double speed){
        m_rightShootMotor.set(speed);
        m_leftShootMotor.set(speed);
        m_containmentMotor.set(speed);
    }

    /**
     * stop spining rollers 
     */
    public void stopRollers(){
        m_targetShootSpeed = 0;
        m_leftShootMotor.getPIDController().setReference(0, ControlType.kVelocity);
        m_rightShootMotor.getPIDController().setReference(0, ControlType.kVelocity);
        m_containmentMotor.getPIDController().setReference(0, ControlType.kVelocity);
    }

    /**
     * stop the motor that controls the angle of the shooter
     */
    public void stopAimMotor(){
        m_aimMotor.stopMotor();
    }

    /**
     * checks if the shooter is ready to shoot
     * @return true if ready to shoot, else false
     */
    public boolean isReadyToShoot() {
        return ((Math.abs(getShooterAngle() - m_targetAngle) <= ShooterValues.AIM_MOTOR_MIN_TOLERANCE) &&
        (Math.abs(getLeftRollerSpeed() - m_targetShootSpeed) <= ShooterValues.SHOOT_SPEED_TOLERANCE)   &&
        (Math.abs(getRightRollersSpeed() - m_targetShootSpeed) <= ShooterValues.SHOOT_SPEED_TOLERANCE))&& 
        (!isReadyToCollect()) &&
        (m_targetShootSpeed != 0.0) &&
        (m_bottomLimitSwitch.get());
    }

    /**
     * checks if ready to collect
     * @return true if ready to collect else false
     */
    public boolean isReadyToCollect(){
        return ((!m_bottomLimitSwitch.get()) || (Math.abs(getShooterAngle()-getTargetAngle()) < ShooterValues.AIM_MOTOR_MIN_TOLERANCE))
         && !(isNoteIn());
    }

    public boolean isNoteIn(){
        // this random function with the weird magic numbers is taken directly from WPILIB to convert voltage to cm
        return (Math.pow(m_noteSensor.getAverageVoltage(), -1.2045) * 27.726) <= ShooterValues.MIN_NOTE_DISTANCE;
    }

}