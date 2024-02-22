package frc.robot.Subsystems;

import java.io.IOException;

import org.opencv.core.Mat;

import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.AHRS.SerialDataType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.ColorSensorV3.ColorSensorMeasurementRate;
import com.revrobotics.ColorSensorV3.ColorSensorResolution;
import com.revrobotics.ColorSensorV3.GainFactor;
import com.revrobotics.ColorSensorV3.RawColor;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.Consts;
import frc.robot.Utils.Funcs;
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
    private Thread m_thread;
    //controls the angle of the shooter
    private CANSparkMax m_aimMotor;
    private double m_targetShooterAngle = ShooterValues.AIM_MOTOR_MIN_ANGLE;
    //sends signal when the shooter is at the top angle
    // private DigitalInput m_topLimitSwitch;
    //sends signal when the shooter is at the bottom angle
    private DigitalInput m_bottomLimitSwitch;
    //detect when the note is inside to shooter 
    private ColorSensorV3 m_colorSensor;
    private ColorMatch m_colorMatcher;
    private AHRS m_gyro;
    private double m_angleOffset = 0; // the offset between the gyro angle and the real angle, checked when limit switch is pressed
    private PIDController m_shooterAnglePID;

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
        
        
        // m_aimMotor.getPIDController().setOutputRange(-ShooterValues.AIM_MOTOR_VOLTAGE_LIMIT, 
        // ShooterValues.AIM_MOTOR_VOLTAGE_LIMIT);
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

        // m_aimMotor.getPIDController().setP(PIDValues.SHOOTER_ANGLE_KP);
        // m_aimMotor.getPIDController().setI(PIDValues.SHOOTER_ANGLE_KI);
        // m_aimMotor.getPIDController().setD(PIDValues.SHOOTER_ANGLE_KD);

        //set idle mode
        m_rightShootMotor.setIdleMode(ShooterValues.RIGHT_SHOOT_IDLE_MODE);
        m_leftShootMotor.setIdleMode(ShooterValues.LEFT_SHOOT_IDLE_MODE);
        m_aimMotor.setIdleMode(ShooterValues.AIM_IDLE_MODE);
        
        

        //gear ratio
        // m_aimMotor.getEncoder().setPositionConversionFactor(ShooterValues.AIM_MOTOR_GEAR_RATIO * 360); //convert to degrees

        // //initialize position in aim motor
        // m_aimMotor.getEncoder().setPosition(Consts.ShooterValues.AIM_MOTOR_MIN_ANGLE);

        m_gyro = new AHRS(I2C.Port.kOnboard);
        m_thread = new Thread(()->{
            while (true) {
                try {
                    m_gyro.setAngleAdjustment((Swerve.getInstance(ChassisValues.USES_ABS_ENCODER).getGyro().getYaw()));
                }
                catch (Exception e) {
                }
            }
        });
        m_thread.setDaemon(true);
        m_thread.start();
        // m_gyro = new AHRS(SerialPort.Port.kUSB1);
        
        // m_targetShooterAngle = getShooterAngle();
        

        m_shooterAnglePID = new PIDController(PIDValues.SHOOTER_ANGLE_KP,
                            PIDValues.SHOOTER_ANGLE_KI, PIDValues.SHOOTER_ANGLE_KD);
        m_shooterAnglePID.setSetpoint(m_targetShooterAngle);
        
        //create limit switches objects
        // m_topLimitSwitch = new DigitalInput(ShooterValues.TOP_LIMIT_SWITCH_ID);
        m_bottomLimitSwitch = new DigitalInput(ShooterValues.BOTTOM_LIMIT_SWITCH_ID);

        //create color sensor object 
        // m_colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
        //create color matcher object 
        m_colorMatcher = new ColorMatch();
        m_colorMatcher.addColorMatch(Color.kOrange);
        m_colorMatcher.addColorMatch(Color.kOrangeRed);
        m_colorMatcher.addColorMatch(Color.kDarkOrange);
        m_colorMatcher.setConfidenceThreshold(ShooterValues.COLOR_SENSOR_CONFIDENCE);
        

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
        SmartDashboard.putNumber("target shooter angle", m_targetShooterAngle);
        SmartDashboard.putNumber("shooter angle", getShooterAngle());
        SmartDashboard.putNumber("shooter pitch", m_gyro.getPitch());
        SmartDashboard.putNumber("shooter yaw", m_gyro.getAngle());
        SmartDashboard.putNumber("shooter roll", m_gyro.getRoll());
        SmartDashboard.putNumber("yaw ruin", (180/Math.PI)*Math.atan2(Math.cos((Math.PI/180)*m_gyro.getRoll()), Math.sin((Math.PI/180)*m_gyro.getPitch())));
         SmartDashboard.putNumber("yaw fix2", m_gyro.getRoll() + m_gyro.getPitch());
        // SmartDashboard.putNumber("shooter z speed", m_gyro.getRawGyroZ());
        // SmartDashboard.putNumber("shooter x speed", m_gyro.getRawGyroX());
        // SmartDashboard.putNumber("shooter y speed", m_gyro.getRawGyroY());
        

        SmartDashboard.putNumber("right rollers speed", getRightRollersSpeed());
        SmartDashboard.putNumber("left rollers speed", getLeftRollerSpeed());
        
        //set encoder position value when touches limit switches
        // if(m_topLimitSwitch.get())
        //     m_aimMotor.getEncoder().setPosition(ShooterValues.AIM_MOTOR_MAX_ANGLE);
        if(!m_bottomLimitSwitch.get()){
            m_angleOffset = ShooterValues.AIM_MOTOR_MIN_ANGLE - getRawShooterAngle();   
        }
            
        SmartDashboard.putBoolean("limit switch", m_bottomLimitSwitch.get());

        //start spining rollers if note is inside the shooter 
        // ColorMatchResult result = m_colorMatcher.matchColor(m_colorSensor.getColor());
        // if(result != null && result.confidence >= ShooterValues.COLOR_SENSOR_CONFIDENCE)
        //     setShootSpeed(getLeftRollerSpeed());

        m_shooterAnglePID.setSetpoint(m_targetShooterAngle);
        
        double output = m_shooterAnglePID.calculate(getShooterAngle());
        output = MathUtil.clamp(output, -ShooterValues.AIM_MOTOR_SPEED_LIMIT, ShooterValues.AIM_MOTOR_SPEED_LIMIT);
        if (m_targetShooterAngle == ShooterValues.AIM_MOTOR_MIN_ANGLE && m_bottomLimitSwitch.get()){
            m_aimMotor.set(-Math.max(0.1, Math.abs(output)));
        }
        else {
            if (Math.abs(output) >= ShooterValues.AIM_MOTOR_MIN_SPEED){
                m_aimMotor.set(output);
            }
            else {
                m_aimMotor.set(0);
            }
        }  

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
        if(DriverStation.getAlliance().get() == Alliance.Blue){
            speakerPos = ShooterValues.BLUE_SPAKER_POS;
        }
        else if(DriverStation.getAlliance().get() == Alliance.Red){
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

        // this makes sure that the accumelated integral from before will not ruin the results if we switch directions
        if (Math.signum(angle - getShooterAngle()) != Math.signum(m_targetShooterAngle - getShooterAngle())) {
            // m_aimMotor.getPIDController().setIAccum(0);
            m_shooterAnglePID.reset();
        }
        m_targetShooterAngle = angle;
        m_shooterAnglePID.setSetpoint(angle);
        //  m_aimMotor.getPIDController().setReference(angle, ControlType.kPosition);
        // //scale kf according to the shooter's angle(watch the cosinus function graph inorder to understand why its here)
        // double ff = Math.cos(Math.toRadians(angle)) * PIDValues.SHOOTER_ANGLE_KF;
        // //activate pid
        // m_aimMotor.getPIDController().setReference(angle, ControlType.kPosition, 0, ff, SparkMaxPIDController.ArbFFUnits.kPercentOut);
    }

    /**
     * 
     * @return current shooter angle
     */
    public double getShooterAngle(){
        // m_gyro.zeroYaw();
        return getRawShooterAngle() + m_angleOffset;
    }

    public double getTargetShooterAngle(){
        return m_targetShooterAngle;
    }

    /**
     * 
     * @return the shooters angle with no offset
     */
    private double getRawShooterAngle(){
        // m_gyro.zeroYaw();
        return -Funcs.modulo(m_gyro.getRoll(), 360);
        // (((Funcs.modulo(m_gyro.getRoll() - 90, 360.0)) <= 180) 
        // ? (180 - m_gyro.getPitch()) : (m_gyro.getPitch())); 
        //return Funcs.convertRotationsToDegrees(m_gyro.getRotation3d().getX() / (2*Math.PI));
    }

    /**
     * Shoot note
     * @param speed - target shoot speed in rpm
     */
    public void shoot(double speed){
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
        //activate velocity pid on right rollers motor controller
        m_rightShootMotor.getPIDController().setReference(speed, ControlType.kVelocity);
        //activate velocity pid on left rollers motor controller
        m_leftShootMotor.getPIDController().setReference(speed, ControlType.kVelocity);
        //activate velocity pid on containment roller motor controller
        m_containmentMotor.getPIDController().setReference(speed, ControlType.kVelocity);
    }

    public void testMotors(double speed){
        m_rightShootMotor.set(speed);
        m_leftShootMotor.set(speed);
        m_containmentMotor.set(speed);
    }

    /**
     * stop spining rollers 
     */
    public void stopRollers(){
        m_leftShootMotor.set(0);
        m_rightShootMotor.set(0);
        m_containmentMotor.set(0);
    }

    /**
     * stop the motor that controls the angle of the shooter
     */
    public void stopAimMotor(){
        m_aimMotor.stopMotor();
    }

    /**
     * tells if note is in
     */
    public boolean isNoteIn(){
        // ColorMatchResult result = m_colorMatcher.matchColor(m_colorSensor.getColor());
        // return result != null;
        return false;
        // if (result == null) {
        //     return false;
        // }

        // return result.confidence  >= ShooterValues.COLOR_SENSOR_CONFIDENCE;
    }


    // public Color getColor(){
    //     SmartDashboard.putBoolean("check",  m_colorSensor.isConnected());
      
    //     return m_colorSensor.getColor();
    // }


}
