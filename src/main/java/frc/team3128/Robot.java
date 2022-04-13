// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team3128;

import java.util.ArrayList;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation.
 */
public class Robot extends TimedRobot {

    public static RobotContainer m_robotContainer = new RobotContainer();
    private Command m_autonomousCommand;
    // private Thread dashboardUpdateThread;

    // private ArrayList<Double> battVoltages = new ArrayList<Double>();
    // public static double voltageRollingAvg = 0;

    @Override
    public void robotInit(){
        LiveWindow.disableAllTelemetry();
    }

    @Override
    public void robotPeriodic(){
        CommandScheduler.getInstance().run();
        m_robotContainer.updateDashboard();
    }

    @Override
    public void autonomousInit() {
        m_robotContainer.init();
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopInit() {
        m_robotContainer.init();
        CommandScheduler.getInstance().cancelAll();
        m_robotContainer.stopDrivetrain();
    }

    @Override
    public void teleopPeriodic() {

    }

    @Override
    public void simulationInit() {
        
    }

    @Override
    public void simulationPeriodic() {

    }
    
    @Override
    public void disabledPeriodic() {

    }
}
