package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Climber;
import frc.robot.Utils.Consts;

public class ClimbWithoutPID extends Command implements Consts {

    double speed;

    public ClimbWithoutPID(double speed){
        this.speed = speed;
    }
    @Override
    public void initialize(){
        addRequirements(Climber.getInstance());
    } 
    
    @Override
    public void execute(){
        Climber.getInstance().climbWithoutPID(speed);
    } 

    @Override
    public boolean isFinished(){
        return (Climber.getInstance().m_climberLimitSwitchOne.get() && Climber.getInstance().m_climberLimitSwitchTwo.get());
    } 

    @Override
    public void end(boolean interrupted){
    } 
}