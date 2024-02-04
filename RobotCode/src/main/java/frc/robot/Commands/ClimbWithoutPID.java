package frc.robot.Commands;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.ClimberSubSystem;
import frc.robot.Utils.Consts;
import frc.robot.Subsystems.ClimberSubSystem;

public class ClimbWithoutPID extends CommandBase implements Consts {

    double speed;

    public ClimbWithoutPID(double speed){
        this.speed = speed;
    }
    @Override
    public void initialize(){
        addRequirements(ClimberSubSystem.getInstance());
    } 
    
    @Override
    public void execute(){
        ClimberSubSystem.getInstance().climbWithoutPID(speed);
    } 

    @Override
    public boolean isFinished(){
        return (ClimberSubSystem.getInstance().m_climberLimitSwitchOne.get() && ClimberSubSystem.getInstance().m_climberLimitSwitchTwo.get());
    } 

    @Override
    public void end(boolean interrupted){
    } 
}