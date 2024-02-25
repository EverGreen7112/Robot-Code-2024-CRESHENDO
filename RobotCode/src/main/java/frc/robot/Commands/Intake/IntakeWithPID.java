package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Utils.Consts;

public class IntakeWithPID extends Command implements Consts {

    private double m_rpm;

    public IntakeWithPID(double rpm){
        this.m_rpm = rpm;
    }
    @Override
    public void initialize(){
        addRequirements(Intake.getInstance());
        addRequirements(Shooter.getInstance());
    }

    @Override
    public void execute(){
        Shooter.getInstance().turnToAngle(ShooterValues.AIM_MOTOR_MIN_ANGLE);
        Shooter.getInstance().pullNote(-m_rpm);
        if (Shooter.getInstance().isReadyToCollect()){
             Intake.getInstance().intakeWithPID(m_rpm);
        } 
        else {
            Intake.getInstance().intakeWithPID(0);
        }
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
