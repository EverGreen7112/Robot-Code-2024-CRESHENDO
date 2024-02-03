package frc.robot.SubSystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.Consts;

public class IntakeSubSystem extends SubsystemBase implements Consts {
    private static IntakeSubSystem m_intakeSubSystem;
    public CANSparkMax m_intakeMotor;

    public IntakeSubSystem(){
        m_intakeMotor = new CANSparkMax(IntakeValues.INTAKE_MOTOR_PORT, MotorType.kBrushless);

        m_intakeMotor.getPIDController().setP(IntakeValues.INATKE_MOTOR_KP);
        m_intakeMotor.getPIDController().setI(IntakeValues.INATKE_MOTOR_KI);
        m_intakeMotor.getPIDController().setD(IntakeValues.INATKE_MOTOR_KD);
        m_intakeMotor.getPIDController().setFF(IntakeValues.INTAKE_MOTOR_FF);

    }

    public static IntakeSubSystem getInstance(){
        if(m_intakeSubSystem == null)
            m_intakeSubSystem = new IntakeSubSystem();
        return m_intakeSubSystem;
    }

    public void IntakeWithPID(double rpm){
        m_intakeMotor.getPIDController().setReference(rpm, ControlType.kVelocity);
    }

    public void IntakeWithoutPID(double speed){
        m_intakeMotor.set(speed);
    }


}
