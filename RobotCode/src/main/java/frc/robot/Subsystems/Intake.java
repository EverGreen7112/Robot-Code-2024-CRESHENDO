package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.Consts;

public class Intake extends SubsystemBase implements Consts {

    private static Intake m_instance;
    
    private CANSparkMax m_intakeMotor;

    public Intake(){
        m_intakeMotor = new CANSparkMax(IntakeValues.INTAKE_MOTOR_PORT, MotorType.kBrushless);

        m_intakeMotor.restoreFactoryDefaults();

        m_intakeMotor.getPIDController().setP(IntakeValues.INTAKE_MOTOR_KP);
        m_intakeMotor.getPIDController().setI(IntakeValues.INATKE_MOTOR_KI);
        m_intakeMotor.getPIDController().setD(IntakeValues.INATKE_MOTOR_KD);
        m_intakeMotor.getPIDController().setFF(IntakeValues.INTAKE_MOTOR_FF);

    }
    
    /**
     * returns the onyl instance of this subsystem
     */
    public static Intake getInstance(){
        if(m_instance == null)
            m_instance = new Intake();
        return m_instance;
    }

    /**
     * spins the motor at the given rpm
     * @param rpm the rpm the motors will spin at
     * 
     *
     */
    public void intakeWithPID(double rpm){
        m_intakeMotor.getPIDController().setReference(rpm, ControlType.kVelocity);
    }

    /**
     * spin the motor at the given speed
     * @param speed speed value given to the motor between -1 and 1
     */
    public void intakeWithoutPID(double speed){
        m_intakeMotor.set(speed);
    }

    /**
     * stops the motors
     */
    public void stopMotor(){
        m_intakeMotor.set(0);
    }


}
