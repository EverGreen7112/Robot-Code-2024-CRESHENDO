package frc.robot.Commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Swerve;
import frc.robot.Utils.Consts;
import frc.robot.Utils.Funcs;
import frc.robot.Utils.Consts.ShooterValues;

public class ShootFromClose extends Command implements Consts{
    
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double currentAngle = Funcs.modulo(Swerve.getInstance(ChassisValues.USES_ABS_ENCODER).getGyro().getAngle(), 360);
        if(currentAngle > 200 && currentAngle < 250){
            //shoot from right
            
        } 
        else if(currentAngle > 100 && currentAngle < 130){
            //shoot from left

        }
        else {
            //shoot from center
            Shooter.getInstance().turnToAngle(114);
            Shooter.getInstance().setShootSpeed(7000, ShooterValues.SPEAKER_SHOOT_SPEED * 1.1 / 3);
        }
        //  new InstantCommand(()->{Shooter.getInstance().setShootSpeed(7000, ShooterValues.SPEAKER_SHOOT_SPEED * 1.1 / 3);})))
    }

    @Override
    public void end(boolean interrupted) {
        Shooter.getInstance().stopRollers();
    }

}
