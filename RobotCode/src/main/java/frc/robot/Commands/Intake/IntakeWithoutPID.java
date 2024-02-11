package frc.robot.Commands.Intake;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake;
import frc.robot.Utils.Consts;

public class IntakeWithoutPID extends Command implements Consts {

    private double m_speed;

    public IntakeWithoutPID(double speed){
        this.m_speed = speed;
    }

    @Override
    public void initialize(){
        addRequirements(Intake.getInstance());
    }

    @Override
    public void execute(){
        Intake.getInstance().intakeWithoutPID(m_speed);
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