package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.Consts;

public class ClimberSubSystem extends SubsystemBase implements Consts {
    private static ClimberSubSystem m_climberSubSystem;
    public CANSparkMax m_climberMotorOne, m_climberMotorTwo;
    public DigitalInput m_climberLimitSwitchOne, m_climberLimitSwitchTwo;

    public ClimberSubSystem() {

        m_climberLimitSwitchOne = new DigitalInput(ClimberValues.CLIMBER_LIMIT_SWITCH_ONE_PORT);
        m_climberLimitSwitchTwo = new DigitalInput(ClimberValues.CLIMBER_LIMIT_SWITCH_TWO_PORT);

        m_climberMotorOne = new CANSparkMax(ClimberValues.CLIMBER_MOTOR_ONE_PORT, MotorType.kBrushless);
        m_climberMotorTwo = new CANSparkMax(ClimberValues.CLIMBER_MOTOR_TWO_PORT, MotorType.kBrushless);

        // restore factory defults

        m_climberMotorOne.restoreFactoryDefaults();
        m_climberMotorOne.restoreFactoryDefaults();

        // set gear ratio

        m_climberMotorOne.getEncoder().setVelocityConversionFactor(ClimberValues.CLIMBER_MOTOR_ONE_GEAR_RATIO);
        m_climberMotorTwo.getEncoder().setVelocityConversionFactor(ClimberValues.CLIMBER_MOTOR_TWO_GEAR_RATIO);

        // set pid values

        m_climberMotorOne.getPIDController().setP(ClimberValues.CLIMBER_MOTOR_ONE_KP);
        m_climberMotorOne.getPIDController().setI(ClimberValues.CLIMBER_MOTOR_ONE_KI);
        m_climberMotorOne.getPIDController().setD(ClimberValues.CLIMBER_MOTOR_ONE_KD);
        m_climberMotorOne.getPIDController().setFF(ClimberValues.CLIMBER_MOTOR_ONE_FF);

        m_climberMotorTwo.getPIDController().setP(ClimberValues.CLIMBER_MOTOR_ONE_KP);
        m_climberMotorTwo.getPIDController().setI(ClimberValues.CLIMBER_MOTOR_ONE_KI);
        m_climberMotorTwo.getPIDController().setD(ClimberValues.CLIMBER_MOTOR_ONE_KD);
        m_climberMotorTwo.getPIDController().setFF(ClimberValues.CLIMBER_MOTOR_TWO_FF);

        // invert motors if necessary

        m_climberMotorOne.setInverted(ClimberValues.IS_MOTOR_ONE_INVERTED);
        m_climberMotorTwo.setInverted(ClimberValues.IS_MOTOR_TWO_INVERTED);

        // put motors on brake

        m_climberMotorOne.setIdleMode(IdleMode.kBrake);
        m_climberMotorTwo.setIdleMode(IdleMode.kBrake);

    }

    /**
     * 
     * @return returns the only instance of this subsystem.
     */
    public static ClimberSubSystem getInstance() {
        if (m_climberSubSystem == null)
            m_climberSubSystem = new ClimberSubSystem();
        return m_climberSubSystem;

    }

    /**
     * Stop both climber motors.
     */
    public void stopClimberMotor() {
        m_climberMotorOne.stopMotor();
        m_climberMotorTwo.stopMotor();

    }

    /**
     * 
     * @param Mode Set both climb motors to give Idle mode(Brake/Coast).
     */
    public void setMode(IdleMode Mode) {
        m_climberMotorOne.setIdleMode(Mode);
        m_climberMotorTwo.setIdleMode(Mode);

    }

    /**
     * 
     * @param speed Moves both climb motors at the given speed without pid
     */
    public void move(double speed) {
        m_climberMotorOne.set(speed);
        m_climberMotorTwo.set(speed);

    }

    /**
     * the function moves the motors at the given rpm using pid.
     * 
     * @param rpm move the motors at the given rpm.
     */
    public void pidClimb(double rpm) {
        m_climberMotorOne.getPIDController().setReference(rpm, ControlType.kVelocity);
        m_climberMotorOne.getPIDController().setReference(rpm, ControlType.kVelocity);

    }

}
