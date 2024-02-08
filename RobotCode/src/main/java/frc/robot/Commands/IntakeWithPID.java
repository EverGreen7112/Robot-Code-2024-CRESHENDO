package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake;
import frc.robot.Utils.Consts;

public class IntakeWithPID extends Command implements Consts {

    private double m_rpm;

    public IntakeWithPID(double rpm){
        this.m_rpm = rpm;
    }
    @Override
    public void initialize(){
        addRequirements(Intake.getInstance());
    }

    @Override
    public void execute(){
        Intake.getInstance().intakeWithPID(m_rpm);
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
