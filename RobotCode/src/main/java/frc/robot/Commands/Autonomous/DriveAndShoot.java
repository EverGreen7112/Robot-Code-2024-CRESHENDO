package frc.robot.Commands.Autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.Commands.Shooter.ShootToSpeaker;
import frc.robot.Commands.Shooter.TurnShooterToSpeaker;
import frc.robot.Commands.Swerve.TurnToSpeaker;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Swerve;
import frc.robot.Utils.Consts;
import frc.robot.Utils.Vector2d;

public class DriveAndShoot extends Command implements Consts{
    private double m_startTime;
    private boolean m_isFinished;
    private boolean m_turned;
    public DriveAndShoot(){
        addRequirements(Swerve.getInstance(ChassisValues.USES_ABS_ENCODER));
    }

    @Override
    public void initialize() {
        m_startTime = System.currentTimeMillis() / 1000.0;
        m_isFinished = false;
        m_turned = false;
        Vector2d speaker2d = Robot.getSpeaker2d();
        Swerve.getInstance(ChassisValues.USES_ABS_ENCODER).setOdometryVals(speaker2d.x, speaker2d.y, 
        Swerve.getInstance(ChassisValues.USES_ABS_ENCODER).getFieldOrientedAngle());
    }
    @Override
    public void execute() {
        double time = System.currentTimeMillis() / 1000.0 - m_startTime;
        
        if(time > 0 && time <= 1){
            Swerve.getInstance(ChassisValues.USES_ABS_ENCODER).driveOriginOriented(new Vector2d(0, -0.8), false);            
        }
        else if (time > 1 && time < 2 && !m_turned) {
            Swerve.getInstance(ChassisValues.USES_ABS_ENCODER).turnBy(180);
            m_turned = true;
        }
        else if(time > 2 && time < 3){
            new TurnToSpeaker().schedule();
            Swerve.getInstance(ChassisValues.USES_ABS_ENCODER).driveOriginOriented(new Vector2d(), false);
        }
        else if(time > 3 && time < 4){
            new TurnShooterToSpeaker().schedule();
            new InstantCommand(() -> {Shooter.getInstance().setShootSpeed(ShooterValues.SPEAKER_SHOOT_SPEED);}).schedule();;
        }
        else if(time > 5 && time < 6 && Shooter.getInstance().isReadyToShoot()){
            new InstantCommand(() -> {Shooter.getInstance().pushNoteToRollers(ShooterValues.CONTAINMENT_SPEED);}).schedule();;
        }
        else{
            Swerve.getInstance(ChassisValues.USES_ABS_ENCODER).driveOriginOriented(new Vector2d(), true);
        }
        
    }

    @Override
    public boolean isFinished() {
        return m_isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        Swerve.getInstance(ChassisValues.USES_ABS_ENCODER).stop();
    }
}
