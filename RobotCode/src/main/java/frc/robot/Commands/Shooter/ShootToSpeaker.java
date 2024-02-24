package frc.robot.Commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shooter;
import frc.robot.Utils.Consts.ShooterValues;

public class ShootToSpeaker extends Command{
    
    @Override
    public void initialize() {
        Shooter.getInstance().pullNote(ShooterValues.CONTAINMENT_SPEED);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Shooter.getInstance().stopRollers();
    }
}
