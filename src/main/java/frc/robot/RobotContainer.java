// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.lang.model.element.ModuleElement;

import  edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;

import frc.robot.subsystems.drive.Drive;

/*
* This class is where the bulk of the robot should be declared.  Since Command-based is a
* "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
* periodic methods (other than the scheduler calls).  Instead, the structure of the robot
* (including subsystems, commands, and button mappings) should be declared here.
*/    

public class RobotContainer {
    // The robot's subsystems
	private static final Drive m_driveSubsystem = new Drive();
    private static final Pivot m_pivot = new Pivot();
    private static final Elevator m_elevator = new Elevator();
    private static final Claw m_claw = new Claw();

	private static final GenericHID demoBoard = new GenericHID(2);
	private static final Joystick joystick = new Joystick(2);

    // The driver's controller
    public static final CommandXboxController m_cont0 = new CommandXboxController(0);
	

    private final static double defaultSpeed = 0.8;
	private final static double slowSpeed = 0.2;
	private static double speedMod = defaultSpeed;

	public static Trigger inputless = new Trigger(()->
		!(demoBoard.getRawButton(1)||
		demoBoard.getRawButton(2)||
		demoBoard.getRawButton(3)||
		demoBoard.getRawButton(4)||
		demoBoard.getRawButton(5)||
		demoBoard.getRawButton(6)));

	public static Trigger cone = new Trigger(()->demoBoard.getRawButton(7));
	public static Trigger cube = cone.negate();
	

	public static Trigger cubeIntakeBack = cube.and(new Trigger(()->demoBoard.getRawButton(2)));
	public static Trigger cubeScoreState = cube.and(new Trigger(()->demoBoard.getRawButton(3)));
	//public static Trigger coneIntakeBack = cone.and(new Trigger(()->demoBoard.getRawButton(1)));
	public static Trigger coneScoreState =  cone.and(new Trigger(()->demoBoard.getRawButton(1)));
	// public static Trigger intake = new Trigger
	// public static Trigger outtake = 

	


    // public static Trigger scoreCone = m_cont0.leftBumper().and(cone).and(slow);
    // public static Trigger scoreCube = m_cont0.a().and(cube);

    // public static Trigger intakingCone = intaking.and(cone);
    // public static Trigger intakingCube = intaking.and(cube);

	
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        // Configure default commands
        m_driveSubsystem.setDefaultCommand(new RunCommand(
                () -> {
                    m_driveSubsystem.drive(
			        m_driveSubsystem.inputDeadband(-joystick.getY()) * slowSpeed,
                    m_driveSubsystem.inputDeadband(-joystick.getX()) * slowSpeed,
                    m_driveSubsystem.inputDeadband(-joystick.getRawAxis(3)+joystick.getRawAxis(2) ) * slowSpeed,
			true);}, m_driveSubsystem));
        // m_claw.setDefaultCommand(new InstantCommand(() -> {m_claw.transitionState();}, m_claw));
        // m_pivot.setDefaultCommand(new InstantCommand(() -> {m_pivot.transitionState();}, m_pivot));
        // m_elevator.setDefaultCommand(new InstantCommand(() -> {m_elevator.transitionState();}, m_elevator));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link org.wpilib.driverstation.GenericHID} or one of its
     * subclasses ({@link
     * org.wpilib.driverstation.Joystick} or {@link XboxController}), and then calling
     * passing it to a
     * {@link JoystickButton}.
     */   
    private void configureButtonBindings() {
        m_cont0.start().onTrue(new InstantCommand(() -> {m_driveSubsystem.zeroHeading();}));
		m_cont0.back().onTrue(new InstantCommand(()->{m_elevator.transitionState();m_pivot.startState();m_claw.transitionState();}));
        // y is for cone


        inputless.whileTrue(transitionState());
        cubeIntakeBack.whileTrue(cubeIntakeBack());
       // cubeIntakeBack.whileTrue(cubeIntakeBack());
       // coneIntakeBack.whileTrue(coneIntakeBack());
        //coneIntakeBack.whileTrue(coneIntakeBack());
        coneScoreState.whileTrue(coneScoreState());
        cubeScoreState.whileTrue(cubeScoreState());
 
        // scoreCone.whileTrue(scoreCone());
        // scoreCube.whileTrue(new RunCommand(() -> {m_claw.outtake();}));

        // intakingCube.whileTrue(new RunCommand(() -> {m_claw.intake();}));
        // intakingCone.whileTrue(new RunCommand(() -> {m_claw.outtake();}, m_claw));
    }

    private Command transitionState()
    {
        // return new InstantCommand(
        //     () -> {
        //        m_pivot.transitionState();
        //        m_elevator.transitionState();
        //         m_claw.transitionState();
        //     });
		// return Commands.run(()-> {m_claw.transitionState(); m_elevator.transitionState();}).andThen(Commands.waitUntil(()->m_elevator.hasReachedTarget())).andThen(new InstantCommand(()->{m_pivot.transitionState();}));
		
		return Commands.sequence(
            Commands.runOnce(() -> {
                m_claw.transitionState();
                m_elevator.transitionState();
            }),
            Commands.waitUntil(m_elevator::hasReachedTarget),
            Commands.runOnce(m_pivot::transitionState)
        );
	}
    private Command cubeIntakeFront()
    {
        // return new InstantCommand(
        //     () -> {
        //         m_pivot.cubeIntakeFront();
        //         m_elevator.cubeIntake();
        //         m_claw.cubeIntakeFront();
        //     });
		// return Commands.run(()-> {m_claw.cubeIntakeFront(); m_pivot.cubeIntakeFront();}).until(()->m_pivot.hasReachedTarget()).andThen(new InstantCommand(()->{m_elevator.cubeIntake();}));
		return Commands.sequence(
            Commands.runOnce(() -> {
                m_claw.cubeIntakeFront(); 
                m_pivot.cubeIntakeFront();
            }),
            Commands.waitUntil(m_pivot::hasReachedTarget),
            Commands.runOnce(m_elevator::cubeIntake)
        );
    }
    private Command cubeIntakeBack()
    {
        // return new InstantCommand(() -> {m_pivot.cubeIntakeBack(); m_claw.cubeIntakeBack();}).until(()->m_pivot.hasReachedTarget()).andThen(new InstantCommand(()->{ m_elevator.cubeIntake();}));
		return Commands.sequence(
            Commands.runOnce(() -> {
                m_pivot.cubeIntakeBack(); 
                m_claw.cubeIntakeBack();
            }),
            Commands.waitUntil(m_pivot::hasReachedTarget),
            Commands.runOnce(m_elevator::cubeIntake)
        );
    }
    private Command coneIntakeFront()
    {
        // return Commands.run(() -> {m_pivot.coneIntakeFront(); m_claw.coneIntakeFront();}).until(()->m_pivot.hasReachedTarget()).andThen(new InstantCommand(()->{m_elevator.coneIntake();})
		return Commands.sequence(
            Commands.runOnce(() -> {
                m_pivot.coneIntakeFront(); 
                m_claw.coneIntakeFront();
            }),
            Commands.waitUntil(m_pivot::hasReachedTarget),
            Commands.runOnce(m_elevator::coneIntake)
        );
    }
    private Command coneIntakeBack()
    {
        // return Commands.run(() -> {m_pivot.coneIntakeBack(); m_claw.coneIntakeBack();}).until(()->m_pivot.hasReachedTarget()).andThen(new InstantCommand(()->{ m_elevator.coneIntake();}));
		return Commands.sequence(
            Commands.runOnce(() -> {
                m_pivot.coneIntakeBack(); 
                m_claw.coneIntakeBack();
            }),
            Commands.waitUntil(m_pivot::hasReachedTarget),
            Commands.runOnce(m_elevator::coneIntake)
        );
		
    }
    private Command coneScoreState()
    {
        // return new InstantCommand(
        //     () -> {
        //         m_pivot.coneScoreState();uuuu
        //         m_claw.coneScoreState();
        //     });
		// return Commands.run(()-> {m_claw.coneScoreState(); m_pivot.coneScoreState();}).andThen(Commands.waitUntil(()->m_pivot.hasReachedTarget())).andThen(new InstantCommand(()->{m_elevator.coneScoreState();}));
		// return Commands.run(()-> {m_claw.coneScoreState(); m_pivot.coneScoreState();})
		// .until(()->!m_pivot.hasReachedTarget())
		// .andThen(Commands.waitUntil(()->m_pivot.hasReachedTarget()))
		// .andThen(new InstantCommand(()->{m_elevator.coneScoreState();}))
		// ;
		return Commands.sequence(
            Commands.runOnce(() -> {
                m_claw.coneScoreState(); 
                m_pivot.coneScoreState();
            }),
            Commands.waitUntil(m_pivot::hasReachedTarget),
            Commands.runOnce(m_elevator::coneScoreState)
        );
    }
    private Command cubeScoreState()
    {
        // // return Commands.run(() -> {m_pivot.cubeScoreState(); m_elevator.cubeScoreState(); m_claw.cubeScoreState();});
		// return Commands.run(()-> {m_claw.cubeScoreState(); m_pivot.cubeScoreState();}).andThen(Commands.waitUntil(()->!m_pivot.hasReachedTarget())).andThen(Commands.waitUntil(()->m_pivot.hasReachedTarget())).andThen(new InstantCommand(()->{m_elevator.cubeScoreState();}));
		// return Commands.run(()-> {m_claw.cubeScoreState(); m_pivot.cubeScoreState();}).until(()->m_pivot.hasReachedTarget()).andThen(new InstantCommand(()->{m_elevator.cubeScoreState();}));
		return Commands.sequence(
            Commands.runOnce(() -> {
                m_claw.cubeScoreState(); 
                m_pivot.cubeScoreState();
            }),
            Commands.waitUntil(m_pivot::hasReachedTarget),
            Commands.runOnce(m_elevator::cubeScoreState)
        );

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */

    public Command getAutonomousCommand() {
		return new InstantCommand();
    //     // Create config for trajectory
    //     TrajectoryConfig config = new TrajectoryConfig(
    //         AutoConstants.kMaxSpeedMetersPerSecond,
    //         AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    //         // Add kinematics to ensure max speed is actually obeyed
    //         .setKinematics(DriveConstants.kDriveKinematics);

    //     // An example trajectory to follow. All units in meters.
    //     Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
    //         // Start at the origin facing the +X direction
    //         new Pose2d(0, 0, new Rotation2d(0)),
    //         // Pass through these two interior waypoints, making an 's' curve path
    //         List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
    //         // End 3 meters straight ahead of where we started, facing forward
    //         new Pose2d(3, 0, new Rotation2d(0)),
    //         config);

    //     var thetaController = new ProfiledPIDController(
    //         AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    //     thetaController.enableContinuousInput(-Math.PI, Math.PI);

    //     // SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
    //         // exampleTrajectory,
    //         // m_driveSubsystem::getPose, // Functional interface to feed supplier
    //         // DriveConstants.kDriveKinematics,

    //         // // Position controllers
    //         // new PIDController(AutoConstants.kPXController, 0, 0),
    //         // new PIDController(AutoConstants.kPYController, 0, 0),
    //         // thetaController,
    //         // m_driveSubsystem::setModuleStates,
    //         // m_driveSubsystem);

    //     // Reset odometry to the starting pose of the trajectory.
    //     m_driveSubsystem.resetOdometry(exampleTrajectory.getInitialPose());

    //     // Run path followuuing command, then stop at the end.
    //     return swerveControllerCommand.andThen(() -> m_driveSubsystem.drive(0, 0, 0, false));
	}

    public static double getPivotPose()
    {
        return m_pivot.getPose();
    }
	public static double getElevatorPose()
	{
		return m_elevator.getPose();
	}
	public static boolean pivotHasReachedTarget()
	{
		return m_pivot.hasReachedTarget();
	}
	
	public static boolean elevatorHasReachedPose()
	{
		return m_elevator.hasReachedTarget();
	}

	public static double getRealTiltPose()
	{
		return m_claw.getRealPose();
	}

	public static double getLength()
	{
		return m_elevator.getLength();
	}
}
