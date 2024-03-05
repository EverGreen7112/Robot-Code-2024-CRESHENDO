package frc.robot.Commands.Intake;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Utils.Consts;

public class IntakeWithoutPID extends Command implements Consts {

    private double m_speed;

    public IntakeWithoutPID(double speed){
        this.m_speed = speed;
    }

    @Override
    public void initialize(){
        addRequirements(Intake.getInstance());
        addRequirements(Shooter.getInstance());
    }

    @Override
    public void execute(){
        Shooter.getInstance().turnToAngle(ShooterValues.AIM_MOTOR_MIN_ANGLE);
        Shooter.getInstance().pullNoteWithoutPID(-m_speed);
        Intake.getInstance().intakeWithoutPID(m_speed);
        // if (Shooter.getInstance().isReadyToCollect()){
        //     Intake.getInstance().intakeWithoutPID(m_speed);
        // }
        // else {
        //     Intake.getInstance().intakeWithoutPID(0);
        // }
    }
    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interrupted){
        Intake.getInstance().stopMotor();
    }
}