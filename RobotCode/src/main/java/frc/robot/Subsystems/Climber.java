package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.Consts;

public class Climber extends SubsystemBase implements Consts {
    
    private static Climber m_instance;
    private CANSparkMax m_climberRightMotor, m_climberLeftMotor;
    // private DigitalInput m_climberRightLimitSwitch, m_climberLeftLimitSwitch;

    public static enum ClimberSide{
        CLIMB_WITH_RIGHT_SIDE,
        CLIMB_WITH_LEFT_SIDE,
        CLIMB_WITH_BOTH_SIDES;
    }


    public Climber() {

        // m_climberRightLimitSwitch = new DigitalInput(ClimberValues.CLIMBER_LIMIT_SWITCH_RIGHT_PORT);
        // m_climberLeftLimitSwitch = new DigitalInput(ClimberValues.CLIMBER_LIMIT_SWITCH_LEFT_PORT);

        m_climberRightMotor = new CANSparkMax(ClimberValues.CLIMBER_MOTOR_RIGHT_PORT, MotorType.kBrushless);
        m_climberLeftMotor = new CANSparkMax(ClimberValues.CLIMBER_MOTOR_LEFT_PORT, MotorType.kBrushless);

        // restore factory defults
        m_climberRightMotor.restoreFactoryDefaults();
        m_climberRightMotor.restoreFactoryDefaults();

        // set gear ratio
        m_climberRightMotor.getEncoder().setVelocityConversionFactor(ClimberValues.CLIMBER_GEAR_RATIO);
        m_climberLeftMotor.getEncoder().setVelocityConversionFactor(ClimberValues.CLIMBER_GEAR_RATIO);

        // set pid values
        m_climberRightMotor.getPIDController().setP(ClimberValues.CLIMBER_KP);
        m_climberRightMotor.getPIDController().setI(ClimberValues.CLIMBER_KI);
        m_climberRightMotor.getPIDController().setD(ClimberValues.CLIMBER_KD);
        m_climberRightMotor.getPIDController().setFF(ClimberValues.CLIMBER_FF);

        m_climberLeftMotor.getPIDController().setP(ClimberValues.CLIMBER_KP);
        m_climberLeftMotor.getPIDController().setI(ClimberValues.CLIMBER_KI);
        m_climberLeftMotor.getPIDController().setD(ClimberValues.CLIMBER_KD);
        m_climberLeftMotor.getPIDController().setFF(ClimberValues.CLIMBER_FF);

        // invert motors if necessary

        m_climberRightMotor.setInverted(ClimberValues.IS_MOTOR_RIGHT_INVERTED);
        m_climberLeftMotor.setInverted(ClimberValues.IS_MOTOR_LEFT_INVERTED);

        // put motors on brake

        m_climberRightMotor.setIdleMode(IdleMode.kBrake);
        m_climberLeftMotor.setIdleMode(IdleMode.kBrake);

        m_climberLeftMotor.getEncoder().setPosition(0);
        m_climberRightMotor.getEncoder().setPosition(0);

    }

    /**
     * 
     * @return returns the only instance of this subsystem.
     */
    public static Climber getInstance() {
        if (m_instance == null)
            m_instance = new Climber();
        return m_instance;
    }

    @Override
    public void periodic(){

        // if(Climber.getInstance().m_climberRightLimitSwitch.get())
            // Climber.getInstance().m_climberRightMotor.stopMotor();
        
        // if(Climber.getInstance().m_climberLeftLimitSwitch.get())
            // Climber.getInstance().m_climberLeftMotor.stopMotor();
        
        
    }

    /**
     * Stop both climber motors.
     */
    public void stopClimberMotors() {
        m_climberRightMotor.stopMotor();
        m_climberLeftMotor.stopMotor();

    }

    public DigitalInput getRightLimitSwitch(){
        // return m_climberRightLimitSwitch;
        return null;
        
    }

    public DigitalInput getLeftLimitSwitch(){
        // return m_climberLeftLimitSwitch;
        return null;
    }

    /**
     * 
     * @param Mode Set both climb motors to give Idle mode(Brake/Coast).
     */
    public void setMode(IdleMode Mode) {
        m_climberRightMotor.setIdleMode(Mode);
        m_climberLeftMotor.setIdleMode(Mode);
    }

    /**
     * 
     * @param speed Moves both climb motors at the given speed without pid
     */
    public void climbRightSideWithoutPid(double speed) {
        // m_climberRightMotor.set(speed);
        if (m_climberRightMotor.getEncoder().getPosition() >= ClimberValues.CLIMBER_FORCE_STOP_TOLERANCE || speed >= 0) {
            m_climberRightMotor.set(speed);
        }
    }
        /**
     * 
     * @param speed Moves both climb motors at the given speed without pid
     */
    public void climbLeftSideWithoutPid(double speed) {
        // m_climberLeftMotor.set(speed);
        if (m_climberLeftMotor.getEncoder().getPosition() >= ClimberValues.CLIMBER_FORCE_STOP_TOLERANCE || speed >= 0) {
            m_climberLeftMotor.set(speed);
        }
        
    }
        /**
     * 
     * @param speed Moves both climb motors at the given speed without pid
     */
    public void climbBothSidesWithoutPid(double speed) {
        climbLeftSideWithoutPid(speed);
        climbRightSideWithoutPid(speed);
    }


    // TODO: add the check to make sure we dont pull at 0 when we use PID for the climber

    /**
     * the function moves the motors at the given rpm using pid.
     * 
     * @param rpm move the motors at the given rpm.
     */
    public void climbRightWithPID(double rpm) {
        m_climberRightMotor.getPIDController().setReference(rpm, ControlType.kVelocity);
    }
        /**
     * the function moves the motors at the given rpm using pid.
     * 
     * @param rpm move the motors at the given rpm.
     */
    public void climbLeftWithPID(double rpm) {
        m_climberLeftMotor.getPIDController().setReference(rpm, ControlType.kVelocity);
    }
        /**
     * the function moves the motors at the given rpm using pid.
     * 
     * @param rpm move the motors at the given rpm.
     */
    public void climbBothWithPID(double rpm) {
        m_climberRightMotor.getPIDController().setReference(rpm, ControlType.kVelocity);
        m_climberLeftMotor.getPIDController().setReference(rpm, ControlType.kVelocity);
    }

}
