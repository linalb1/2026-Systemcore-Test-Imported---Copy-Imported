// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
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

    // The driver's controller
    public static final CommandXboxController m_cont0 = new CommandXboxController(0);

    private final static double defaultSpeed = 0.8;
	private final static double slowSpeed = 0.2;
	private static double speedMod = defaultSpeed;

    
    public static Trigger slow = m_cont0.leftTrigger(0.3);

    public static Trigger cone = m_cont0.y();
    public static Trigger cube = cone.negate();

    public static Trigger front = m_cont0.rightTrigger(0.2);

    public static Trigger intaking = m_cont0.rightBumper();

    public static Trigger cubeIntakeFront = cube.and(front).and(intaking).and(slow);
    public static Trigger cubeIntakeBack = cube.and(front).negate().and(intaking).and(slow);

    public static Trigger coneIntakeFront = cone.and(front).and(intaking).and(slow);
    public static Trigger coneIntakeBack = cone.and(front).negate().and(intaking).and(slow);

    public static Trigger coneScoreState = cone.and(intaking).negate().and(slow);
    public static Trigger cubeScoreState = cube.and(intaking).negate().and(slow);

    public static Trigger aligning = m_cont0.leftStick();     

    public static Trigger scoreCone = m_cont0.leftBumper().and(cone).and(slow);
    public static Trigger scoreCube = m_cont0.leftBumper().and(cube).and(slow);

    public static Trigger intakingCone = intaking.and(cone);
    public static Trigger intakingCube = intaking.and(cube);

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
			        m_driveSubsystem.inputDeadband(-m_cont0.getLeftY()) * speedMod,
                    m_driveSubsystem.inputDeadband(-m_cont0.getLeftX()) * speedMod,
                    m_driveSubsystem.inputDeadband(-m_cont0.getRightX()) * speedMod,
			true);}, m_driveSubsystem));
        m_claw.setDefaultCommand(new InstantCommand(() -> {m_claw.transitionState();}, m_claw));
        m_pivot.setDefaultCommand(new InstantCommand(() -> {m_pivot.transitionState();}, m_pivot));
        m_elevator.setDefaultCommand(new InstantCommand(() -> {m_elevator.transitionState();}, m_elevator));
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

        // y is for cone
        slow.onTrue(new InstantCommand(() -> {speedMod = slowSpeed;}))
			.onFalse(new InstantCommand(()-> {speedMod = defaultSpeed;}));

        slow.whileFalse(transitionState());
        cubeIntakeFront.whileTrue(cubeIntakeFront());
        cubeIntakeBack.whileTrue(cubeIntakeBack());
        coneIntakeFront.whileTrue(coneIntakeFront());
        coneIntakeBack.whileTrue(coneIntakeBack());
        coneScoreState.whileTrue(coneScoreState());
        cubeScoreState.whileTrue(cubeScoreState());

        scoreCone.whileTrue(scoreCone());
        scoreCube.whileTrue(scoreCube());

        intakingCube.whileTrue(new RunCommand(() -> {m_claw.intake();}, m_claw));
        intakingCone.whileTrue(new RunCommand(() -> {m_claw.outtake();}, m_claw));
    }

    private Command transitionState()
    {
        return new InstantCommand(
            () -> {
                m_pivot.transitionState();
                m_elevator.transitionState();
                m_claw.transitionState();
            }, m_claw, m_pivot, m_elevator);
    }
    private Command cubeIntakeFront()
    {
        return new InstantCommand(
            () -> {
                m_pivot.cubeIntakeFront();
                m_elevator.cubeIntake();
                m_claw.cubeIntakeBack();
            }, m_claw, m_pivot, m_elevator);
    }
    private Command cubeIntakeBack()
    {
        return new InstantCommand(
            () -> {
                m_pivot.cubeIntakeBack();
                m_elevator.cubeIntake();
                m_claw.cubeIntakeBack();
            }, m_claw, m_pivot, m_elevator);
    }
    private Command coneIntakeFront()
    {
        return new InstantCommand(
            () -> {
                m_pivot.coneIntakeFront();
                m_elevator.coneIntake();
                m_claw.coneIntakeBack();
            }, m_claw, m_pivot, m_elevator);
    }
    private Command coneIntakeBack()
    {
        return new InstantCommand(
            () -> {
                m_pivot.coneIntakeBack();
                m_elevator.coneIntake();
                m_claw.coneIntakeBack();
            }, m_claw, m_pivot, m_elevator);
    }
    private Command coneScoreState()
    {
        return new InstantCommand(
            () -> {
                m_pivot.coneScoreState();
                m_elevator.coneScoreState();
                m_claw.coneScoreState();
            }, m_claw, m_pivot, m_elevator);
    }
    private Command cubeScoreState()
    {
        return new InstantCommand(
            () -> {
                m_pivot.cubeScoreState();
                m_elevator.cubeScoreState();
                m_claw.cubeScoreState();
            }, m_claw, m_pivot, m_elevator);
    }
    private Command scoreCone()
    {
        return new InstantCommand(
            () -> {
                m_claw.intake();
            }, m_claw, m_pivot, m_elevator);
    }
    private Command scoreCube()
    {
        return new InstantCommand(
            () -> {
                m_claw.outtake();
            }, m_claw, m_pivot, m_elevator);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */

    //   public Command getAutonomousCommand() {
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


    public static double getPivotPose()
    {
        return m_pivot.getPose();
    }
}
  