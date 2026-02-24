
package frc.robot.Subsystems.Swerve;



import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.MAutils.PoseEstimation.PoseEstimator;
import com.MAutils.Swerve.Utils.PPHolonomicDriveController;
import com.fasterxml.jackson.databind.node.POJONode;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;

import com.pathplanner.lib.util.FileVersionException;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SwerveAutoFollower {
    


    private static Swerve swerve = Swerve.getInstance();
    private static RobotConfig config;

    public SwerveAutoFollower() {
        
        try{
            config = RobotConfig.fromGUISettings();
          } catch (Exception e) {
              e.printStackTrace();
          }

        AutoBuilder.configure(
            () -> PoseEstimator.getCurrentPose() ,
            pose -> PoseEstimator.resetPose(pose), 
            () -> swerve.getChassisSpeeds(),
            (speeds , feedforwards) -> swerve.drive(speeds),
            new PPHolonomicDriveController(new PIDConstants(0), new PIDConstants(0)),
                config,
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                  return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
              }, swerve);


        //Pathfinding.setPathfinder(new LocalADStar());

        
    }

    public static Command buildAuto(String autoName) {
        return new PathPlannerAuto(autoName);
    }

    public static Command followPath(String path) {
        try {
            return AutoBuilder.followPath(PathPlannerPath.fromPathFile(path));
        } catch (FileVersionException | IOException | ParseException e) {
            e.printStackTrace();
        }
        return new InstantCommand();
    }

    public static Command pathFindToPose(Pose2d targetPose , PathConstraints constraints) {
        return AutoBuilder.pathfindToPose(targetPose, constraints);
    }
}