package frc.robot.Commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shooter;

public class ShootToSpeaker extends Command{
    
    @Override
    public void execute() {
        Shooter.getInstance().shootNote(0);
    }    


    

}
