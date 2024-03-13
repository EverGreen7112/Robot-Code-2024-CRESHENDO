package frc.robot.Commands.Swerve;

import java.lang.reflect.Array;
import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Swerve;
import frc.robot.Utils.Consts;
import frc.robot.Utils.SwervePoint;

public class TurnTo extends Command implements Consts{
    private double m_angle;
    
    
    public TurnTo(double angle){
        m_angle = angle;
    }

    @Override
    public void initialize() {
        ArrayList<SwervePoint> point = new ArrayList<>();
        point.add(new SwervePoint(Swerve.getInstance(ChassisValues.USES_ABS_ENCODER).getX(), Swerve.getInstance(ChassisValues.USES_ABS_ENCODER).getY(), m_angle));
        FollowRoute route = new FollowRoute(point);
        route.schedule();
    }


}
