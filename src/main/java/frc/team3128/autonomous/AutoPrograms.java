package frc.team3128.autonomous;

import java.io.IOException;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.LinkedHashMap;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team3128.commands.CmdAlign;
import frc.team3128.commands.CmdExtendIntake;
import frc.team3128.commands.CmdExtendIntakeAndRun;
import frc.team3128.commands.CmdHopperShooting;
import frc.team3128.commands.CmdInPlaceTurn;
import frc.team3128.commands.CmdRetractHopper;
import frc.team3128.commands.CmdReverseIntake;
import frc.team3128.commands.CmdShootDist;
import frc.team3128.commands.CmdShootRPM;
import frc.team3128.common.hardware.limelight.Limelight;
import frc.team3128.common.narwhaldashboard.NarwhalDashboard;
import frc.team3128.subsystems.NAR_Drivetrain;
import frc.team3128.subsystems.Shooter;
import frc.team3128.subsystems.Shooter.ShooterState;
import frc.team3128.subsystems.Intake;
import frc.team3128.subsystems.Hopper;
import frc.team3128.subsystems.Hood;
import frc.team3128.Constants.*;


public class AutoPrograms {
    
    private class AutoInfo {
        public Command command;
        public Pose2d initialPose;

        public AutoInfo(Command command, Pose2d initialPose) {
            this.command = command;
            this.initialPose = initialPose;
        }
    }

    private NAR_Drivetrain drive;
    private Shooter shooter;
    private Intake intake;
    private Hopper hopper;
    private Hood hood;
    private Limelight shooterLimelight;

    private HashMap<String, Trajectory> trajectories;
    private LinkedHashMap<String, AutoInfo> autoMap;

    private Command auto_1Ball;
    private Command auto_2Ball;
    private Command auto_3Ball180;
    private Command auto_S2H1;
    private Command auto_S2H2;
    private Command auto_4Ball180;
    private Command auto_5Ball180;
    private Command auto_Billiards;

    public AutoPrograms(NAR_Drivetrain drive, Shooter shooter, Intake intake, Hopper hopper, Hood hood, Limelight shooterLimelight) {
        this.drive = drive;
        this.shooter = shooter;
        this.intake = intake;
        this.hopper = hopper;
        this.hood = hood;
        this.shooterLimelight = shooterLimelight;

        loadTrajectories();
        initAutos();
        initAutoSelector();
    }

    private void loadTrajectories() {
        trajectories = new HashMap<String, Trajectory>();
        
        String[] trajectoryNames = {
            "3_Ball_good",
            "S2H2_i",
            "S2H2_ii",
            "S2H1",
            "S2H2_iii",
            "S2H2_iv",
            "4Ball_Terminal180_i",
            "4Ball_Terminal180_ii",
            "Terminal2Tarmac",
            "Tarmac2Terminal",
            "Billiards_i",
            "Billiards_ii",
        };

        for (String trajectoryName : trajectoryNames) {
            Path path = Filesystem.getDeployDirectory().toPath().resolve("paths").resolve(trajectoryName + ".wpilib.json");
            try {
                trajectories.put(trajectoryName, TrajectoryUtil.fromPathweaverJson(path));
            } catch (IOException ex) {
                DriverStation.reportError("IOException loading trajectory " + trajectoryName, true);
            }
        } 
    }

    private void initAutos() {
        autoMap = new LinkedHashMap<String, AutoInfo>();

        auto_1Ball = new SequentialCommandGroup(
                            alignShootCmd(),

                            trajectoryCmd(Trajectories.driveBack30In)
        );

        auto_2Ball = new SequentialCommandGroup(
                            new ParallelDeadlineGroup(
                                trajectoryCmd(Trajectories.twoBallTraj), 
                                new CmdExtendIntakeAndRun(intake, hopper)
                            ),

                            new CmdInPlaceTurn(drive, 180),

                            alignShootCmd()
        );

        auto_3Ball180 = new SequentialCommandGroup(
                            shootCmd(), // shootCmd(2800, 19)

                            new CmdInPlaceTurn(drive, 180),

                            new ParallelDeadlineGroup(
                                trajectoryCmd("3_Ball_good"), 
                                new CmdExtendIntakeAndRun(intake, hopper)
                            ),

                            new CmdInPlaceTurn(drive, -50),

                            alignShootCmd() // shootCmd(3340, 2.5)
        );

        auto_S2H1 = new SequentialCommandGroup(

                            //drive and intake ball
                            new ParallelDeadlineGroup(
                                trajectoryCmd("S2H2_i"),
                                new CmdExtendIntakeAndRun(intake, hopper)
                            ),

                            //turn and shoot
                            new CmdInPlaceTurn(drive, 180),

                            alignShootCmd(),

                            //turn and hoard first ball
                            new CmdInPlaceTurn(drive, 90),
                            new ParallelDeadlineGroup(
                                trajectoryCmd("S2H2_ii"),
                                new CmdExtendIntakeAndRun(intake, hopper)
                            ),

                            //drive behind hub
                            new CmdInPlaceTurn(drive, -90),
                            trajectoryCmd("S2H1"),

                            //outtake balls behind hub
                            new CmdExtendIntake(intake),
                            new CmdReverseIntake(intake, hopper)


        );   

        auto_S2H2 = new SequentialCommandGroup(

                            //drive and intake ball
                            new ParallelDeadlineGroup(
                                trajectoryCmd("S2H2_i"),
                                new CmdExtendIntakeAndRun(intake, hopper)
                            ),

                            //turn and shoot
                            new CmdInPlaceTurn(drive, 180),
                            shootCmd(),

                            //turn and hoard first ball
                            new CmdInPlaceTurn(drive, 90),
                            new ParallelDeadlineGroup(
                                trajectoryCmd("S2H2_ii"),
                                new CmdExtendIntakeAndRun(intake, hopper)
                            ),

                            // turn and hoard second ball
                            new CmdInPlaceTurn(drive, 180),
                            new ParallelDeadlineGroup(
                                trajectoryCmd("S2H2_iii"), 
                                new CmdExtendIntakeAndRun(intake, hopper)),
                            
                            //hide ball behinde hub
                            trajectoryCmd("S2H2_iv"),
                            new CmdExtendIntake(intake),
                            new CmdReverseIntake(intake, hopper)

        );

        auto_4Ball180 = new SequentialCommandGroup(
                            //drive and intake 1 ball
                            new ParallelDeadlineGroup(
                                trajectoryCmd("4Ball_Terminal180_i"),  
                                new CmdExtendIntakeAndRun(intake, hopper)),

                            //turn and shoot 2 balls
                            new CmdInPlaceTurn(drive, 180),
                            shootCmd(),

                            //drive to ball and terminal and intake
                            new ParallelDeadlineGroup(
                                trajectoryCmd("4Ball_Terminal180_ii"), 
                                new CmdExtendIntakeAndRun(intake, hopper)),
                            new CmdExtendIntakeAndRun(intake, hopper).withTimeout(1),

                            //drive to tarmac and shoot
                            trajectoryCmd("Terminal2Tarmac"),
                            new CmdInPlaceTurn(drive, 180),
                            alignShootCmd()

        );

        auto_5Ball180 = new SequentialCommandGroup(
                            
                            //shoot preloaded
                            shootCmd(), // shootCmd(2800, 19)

                            //turn and intake next 2 balls
                            new CmdInPlaceTurn(drive, 180),
                            new ParallelDeadlineGroup(
                                trajectoryCmd("3_Ball_good"), 
                                new CmdExtendIntakeAndRun(intake, hopper)
                            ),

                            //shoot 2 balls
                            new CmdInPlaceTurn(drive, -50),
                            shootCmd(),

                            //turn and go to terminal
                            new CmdInPlaceTurn(drive, 180),
                            trajectoryCmd("Tarmac2Terminal"),

                            //intake 2 balls
                            new CmdExtendIntakeAndRun(intake, hopper).withTimeout(2),

                            //return to tarmac and shoot
                            trajectoryCmd("Terminal2Tarmac"),
                            new CmdInPlaceTurn(drive, 180),
                            alignShootCmd()

        );

        auto_Billiards = new SequentialCommandGroup (
                            // initial position: (6.8, 6.272, 45 deg - should be approx. pointing straight at the ball to knock)
                            new SequentialCommandGroup(
                                new CmdExtendIntake(intake),
                                new CmdReverseIntake(intake, hopper)
                            ).withTimeout(2),

                            new CmdInPlaceTurn(drive, 70),

                            new ParallelDeadlineGroup(
                                trajectoryCmd("Billiards_i"),
                                new CmdExtendIntakeAndRun(intake, hopper)
                            ),

                            new CmdInPlaceTurn(drive, 55),

                            shootCmd(1000, 28),

                            new ParallelDeadlineGroup(
                                trajectoryCmd("Billiards_ii"),
                                new CmdExtendIntakeAndRun(intake, hopper)
                            ),

                            new CmdInPlaceTurn(drive, 96),

                            alignShootCmd()
        );

        autoMap.put("1 Ball", new AutoInfo(auto_1Ball, Trajectories.driveBack30In.getInitialPose()));
        autoMap.put("2 Ball", new AutoInfo(auto_2Ball, Trajectories.twoBallTraj.getInitialPose()));
        autoMap.put("3 Ball", new AutoInfo(auto_3Ball180, inverseRotation(trajectories.get("3_Ball_good").getInitialPose())));
        autoMap.put("4 Ball", new AutoInfo(auto_4Ball180, trajectories.get("4Ball_Terminal180_i").getInitialPose()));
        autoMap.put("5 Ball", new AutoInfo(auto_5Ball180, inverseRotation(trajectories.get("3_Ball_good").getInitialPose())));
        autoMap.put("S2H1", new AutoInfo(auto_S2H1, trajectories.get("S2H2_i").getInitialPose()));
        autoMap.put("S2H2", new AutoInfo(auto_S2H2, trajectories.get("S2H2_i").getInitialPose()));
        autoMap.put("Billiards", new AutoInfo(auto_Billiards, new Pose2d(6.8, 6.272, Rotation2d.fromDegrees(45)))); // alternatively, 70 deg. CW from the initial position of Billiards_i
    
    }

    private void initAutoSelector() {
        for (String key : autoMap.keySet()) {
            NarwhalDashboard.addAuto(key, autoMap.get(key).command);
        }
    }

    public Command getAutonomousCommand() {
        String selectedAutoName = NarwhalDashboard.getSelectedAutoName();

        if (selectedAutoName == null) {
            return null;
        }

        AutoInfo selectedInfo = autoMap.get(selectedAutoName);
        drive.resetPose(selectedInfo.initialPose);

        return selectedInfo.command;
    }

    // Helpers to define common commands used in autos
    private Command trajectoryCmd(String trajName) {
        return trajectoryCmd(trajectories.get(trajName));
    }

    private Command trajectoryCmd(Trajectory traj) {
        return new RamseteCommand(traj, 
                            drive::getPose,
                            new RamseteController(DriveConstants.RAMSETE_B, DriveConstants.RAMSETE_ZETA),
                            new SimpleMotorFeedforward(DriveConstants.kS,
                                                        DriveConstants.kV,
                                                        DriveConstants.kA),
                            DriveConstants.DRIVE_KINEMATICS,
                            drive::getWheelSpeeds,
                            new PIDController(DriveConstants.RAMSETE_KP, 0, 0),
                            new PIDController(DriveConstants.RAMSETE_KP, 0, 0),
                            drive::tankDriveVolts,
                            drive).andThen(() -> drive.stop());
    }

    private SequentialCommandGroup shootCmd() {
        return new SequentialCommandGroup(
            new CmdRetractHopper(hopper).withTimeout(0.5),
            new InstantCommand(() -> shooter.setState(ShooterState.UPPERHUB)),
            new InstantCommand(() -> shooterLimelight.turnLEDOn()),
            new ParallelCommandGroup(
                new CmdHopperShooting(hopper, shooter::isReady),
                new CmdShootDist(shooter, hood, shooterLimelight)
            ).withTimeout(2)
        );
    }
    
    private SequentialCommandGroup shootCmd(int RPM, double angle) {
        return new SequentialCommandGroup(
            new CmdRetractHopper(hopper).withTimeout(0.5),
            new InstantCommand(() -> shooter.setState(ShooterState.UPPERHUB)),
            new InstantCommand(() -> shooterLimelight.turnLEDOn()),
            new ParallelCommandGroup(
                new InstantCommand(() -> hood.startPID(angle)),
                new CmdHopperShooting(hopper, shooter::isReady),
                new CmdShootRPM(shooter, RPM)
            ).withTimeout(2)
        );
    }

    private SequentialCommandGroup alignShootCmd() {
        return new SequentialCommandGroup(
            new CmdRetractHopper(hopper).withTimeout(0.5),
            new InstantCommand(() -> shooter.setState(ShooterState.UPPERHUB)),
            new InstantCommand(shooterLimelight::turnLEDOn),
            new ParallelCommandGroup(
                new CmdAlign(drive, shooterLimelight),
                new CmdHopperShooting(hopper, shooter::isReady),
                new CmdShootDist(shooter, hood, shooterLimelight)
            ).withTimeout(5),
            new InstantCommand(shooterLimelight::turnLEDOff)
        );
    }

    private Pose2d inverseRotation(Pose2d pose) {
        return new Pose2d(pose.getTranslation(), pose.getRotation().unaryMinus());
    }

}
