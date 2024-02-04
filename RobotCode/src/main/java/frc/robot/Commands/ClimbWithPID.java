package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Climber;
import frc.robot.Utils.Consts;

public class ClimbWithPID extends Command implements Consts {

    double rpm;

    public ClimbWithPID(double rpm){
        this.rpm = rpm;
    }
    @Override
    public void initialize(){
        addRequirements(Climber.getInstance());
    } 
    
    @Override
    public void execute(){
        Climber.getInstance().climbWithPID(rpm);
    } 

    @Override
    public boolean isFinished(){
        return (Climber.getInstance().m_climberLimitSwitchOne.get() && Climber.getInstance().m_climberLimitSwitchTwo.get());
    } 

    @Override
    public void end(boolean interrupted){
    } 
}
