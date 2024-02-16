package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Climber.ClimberSide;
import frc.robot.Utils.Consts;

public class ClimbWithoutPID extends Command implements Consts {

    private double speed;
    private ClimberSide climbSide;
    
    public ClimbWithoutPID(double speed, ClimberSide climberSide){
        this.speed = speed;
        this.climbSide = climberSide;
    }

    @Override
    public void initialize(){
        addRequirements(Climber.getInstance());
         switch(climbSide){
            case CLIMB_WITH_RIGHT_SIDE:
                Climber.getInstance().climbRightSideWithoutPid(speed);
                break;
            case CLIMB_WITH_LEFT_SIDE:
                Climber.getInstance().climbLeftSideWithoutPid(speed);
                break;
            case CLIMB_WITH_BOTH_SIDES:
                Climber.getInstance().climbBothSidesWithoutPid(speed);
        }
    } 
    
    @Override
    public void execute(){} 

    @Override
    public boolean isFinished(){
        return false;
    } 

    @Override
    public void end(boolean interrupted){
        Climber.getInstance().stopClimberMotors();
    } 
}