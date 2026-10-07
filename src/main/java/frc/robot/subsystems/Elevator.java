// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Configs;
import frc.robot.RobotContainer;
import frc.robot.Constants.Ports;
import frc.robot.util.DIO;

public class Elevator extends SubsystemBase {
	private SparkFlex motorL = new SparkFlex( Ports.elevatorL, MotorType.kBrushless);
	private SparkFlex motorR = new SparkFlex( Ports.elevatorR, MotorType.kBrushless);

	private ArmFeedforward ff = new ArmFeedforward(0, 0, 0);
	private SparkClosedLoopController controller = motorR.getClosedLoopController();

	private boolean topLimit = DIO.getInputs()[9];
	private boolean botLimit = DIO.getInputs()[8];

	public Trigger isDown = new Trigger(()-> botLimit);
	
	public Trigger isUp = new Trigger(()-> topLimit);

	/** Creates a new Elevator. */
	public Elevator() 
	{
		motorL.configure(Configs.elevatorL, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		motorR.configure(Configs.elevatorR, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		isDown.onTrue(new InstantCommand(()->{motorR.getEncoder().setPosition(0);}));
		isUp.onTrue(new InstantCommand(()->{motorL.stopMotor();motorR.stopMotor();motorR.getEncoder().setPosition(73.5);}));
	}

	public enum ElevatorState
	{
		TRANSITION(0),
		CONESCORE(100),
		CUBESCORE(100),
		CONEINTAKE(70),
		CUBEINTAKE(70);

		public double position;
		
		private ElevatorState(double position)
		{
			this.position = position;
		} 
	}
	private ElevatorState state = ElevatorState.TRANSITION;

	@Override
	public void periodic() 
	{
		topLimit = DIO.getInputs()[9];
		botLimit = DIO.getInputs()[8];
		controller.setSetpoint(state.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, Math.sin(Math.toRadians(RobotContainer.getPivotPose()))*ff.calculate(motorR.getEncoder().getPosition(), 0));


		SmartDashboard.putNumber("elevator pos", motorR.getEncoder().getPosition());
		SmartDashboard.putBoolean("down",botLimit);
		SmartDashboard.putBoolean("up",topLimit);
	}

	

	public void transitionState()
	{
		state = ElevatorState.TRANSITION;
	}
	public void scoreConeState()
	{
		state = ElevatorState.CONESCORE;
	}
	public void cubeIntake()
	{
		state = ElevatorState.CUBEINTAKE;
	}
	public void coneIntake()
	{
		state = ElevatorState.CONEINTAKE;
	}
	public void coneScoreState()
	{
		state = ElevatorState.CONESCORE;
	}
	public void cubeScoreState()
	{
		state = ElevatorState.CUBESCORE;
	}

}
