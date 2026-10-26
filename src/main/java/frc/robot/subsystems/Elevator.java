// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.lang.model.util.ElementScanner14;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Configs;
import frc.robot.RobotContainer;
import frc.robot.Constants.Ports;

public class Elevator extends SubsystemBase {
	private SparkFlex motorL = new SparkFlex(Ports.elevatorL, MotorType.kBrushless);
	private SparkFlex motorR = new SparkFlex(Ports.elevatorR, MotorType.kBrushless);

	private SparkClosedLoopController controller = motorR.getClosedLoopController();

	private DigitalInput topInner = new DigitalInput(5);
	private DigitalInput topOuter = new DigitalInput(4);
	private DigitalInput botInner = new DigitalInput(2);
	private DigitalInput botOuter = new DigitalInput(3);


	// 
	public Trigger isDown = new Trigger(()-> !botInner.get() && !botOuter.get());
	
	public Trigger isUp = new Trigger(()-> !topInner.get() && !topOuter.get());

	/** Creates a new Elevator. */
	public Elevator() 
	{
		motorL.configure(Configs.elevatorL, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		motorR.configure(Configs.elevatorR, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		// isDown.onTrue(new InstantCommand(()->{motorR.getEncoder().setPosition(0);}));
		// isUp.onTrue(new InstantCommand(()->{motorL.stopMotor();motorR.stopMotor();motorR.getEncoder().setPosition(73.5);}));
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

	private static double kG = 10;
      
	@Override
	public void periodic() 
	{
		controller.setSetpoint(0, ControlType.kPosition, ClosedLoopSlot.kSlot0, Math.sin(Math.toRadians(RobotContainer.getPivotPose())*kG));

		SmartDashboard.putNumber("elevator pos", motorR.getEncoder().getPosition());
		SmartDashboard.putBoolean("dio 5", !topInner.get());
		SmartDashboard.putBoolean("dio 4", !topOuter.get());
		SmartDashboard.putBoolean("dio 2", !botInner.get());
		SmartDashboard.putBoolean("dio 3", !botOuter.get());
		SmartDashboard.putBoolean("down",isDown.getAsBoolean());
		SmartDashboard.putBoolean("up",isUp.getAsBoolean());

		if(!botInner.get() && !botOuter.get())
			motorR.getEncoder().setPosition(0);
		else if(!topInner.get() && !topOuter.get())
			{
				// motorL.stopMotor();motorR.stopMotor();
				motorR.getEncoder().setPosition(100);}
				
		XboxController a = new XboxController(1); // TODO
		if(a.getLeftStickButton())
			motorR.set(-0.5);
		else if(a.getBackButton())
			motorR.getEncoder().setPosition(0);
	}


	private final double a = 0.251243;

	private final double b = 32.64786;

	public double getLength()
	{
		return a * motorR.getEncoder().getPosition() + b;
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
