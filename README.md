# Offseason 2024

## Intake (Stav)
### Implement the Subsystem
- Include definitions for the motor controller and proximity sensors switches.
- Create methods for basic PercentVBus rotation of the motor, as well as a method for accessing the proximity sensors switches state.
- Add dashboard display of the proximity sensors switches state.
- Create a command to rotate the intake motor based on the XboxController.

### Guidelines:
#### Subsystem Creation
- Define all motor controllers and sensors used in the system.
- Construct these components in the constructor.
- Remember to configure the controller properly.
    - At the very least, reset to factory default.
- You will need to have the following set of methods:
    - A way to rotate the motor based on PercentVBus.
        - You should have constant speeds for in and out (in and out doesn't have to be the same).
    - A way to stop the motor rotation.
    - A way to access proximity sensors switches' information.
- Remember to add print of sensors information to the dashboard.

#### Create Command
- You'll need a command to run your system with an Xbox controller.
- Create a command (or more) and attach them to buttons. At the very least, one button needs to pull note in and one out.
- Configure it so that holding the buttons is required. This eliminates the need for `isFinished` for the moment.
- Don't use the proximity sensors switches in `isFinished` for now.

#### Testing
- Make sure the system moves as expected in both speed and direction.
- Check different speeds for note in and note out. Find optimal speeds and note them in your code.
- Try inserting the note from different positions and directions to make sure the note is collected well.
- Test the proximity sensors switches to make sure it works (use Shuffleboard to view its state).
- Push in a note and see when the proximity sensors switches detects the note.
- Pull the note out and see when the proximity sensors switches no longer detect the note.

### Requirements:
#### Finished Subsystem Code
- Rotate in and out capability with PercentVBus and stop.
    - In and out done at a constant speed.
    - Speed should be determined and tested (not arbitrary).
- Methods to access sensor information.
    - Proximity sensors switches.
- Dashboard prints of sensor information.
- Accurate sensor information.

#### Command
- A command (or more) to rotate the intake in and out.
    - The commands should not use the proximity sensors switches for `isFinished` in this phase.

#### Robot Code
- Code in the robot class that creates the system and runs the command.
    - Attach commands to buttons such that:
        - Holding `RB` pulls note in.
        - Holding `LB` pushes note out.



## Arm (Niv)
### Implement the Subsystem
- Include definitions for the motor controller and limit switch.
- Create methods for basic PercentVBus rotation of the motor, as well as a method for accessing the limit switch state.
- Add dashboard display of the limit switch state.
- Create a command to raise and lower the Arm based on the XboxController.

### Guidelines:
#### Subsystem Creation
- Define all motor controllers (Two Spark maxs) and sensors (Two hardware limit switches and one Through Bore Encoder) used in the system.
- Construct these components in the constructor.
- Remember to configure the controller properly.
    - At the very least, reset to factory default.
    - Then, configure the motor’s controllers to be on break mode and with current limit of 60 amp.  
- You will need to have the following set of methods:
    - A way to rotate the motor based on PercentVBus.
        - You should have constant speeds for raising and lowering the arm. They can (and even should) be different. 
    - A way to stop the motors rotation.
    - A way to access limit switch information.
    - A way to access the angle of the system, using the Through Bore Encoder.
- Remember to add print of sensors information to the dashboard.
- 
#### Create Command
- You'll need a command to run your system with an Xbox controller.
- Create a command (or more) and attach them to buttons. At the very least, one button needs to raise the arm, and one should lower it.
- Configure it so that holding the buttons is required. This eliminates the need for isFinished for the moment.
- Don't use the limit switch in isFinished for now.
  
### Testing
- Make sure the system moves as expected in both speeds and directions.
- Check different speeds for razing and lowering the arm. Find optimal speeds and note them in your code.
- Test the limit switch to make sure it works (use Shuffleboard to view its state).

### Requirements:
#### Finished Subsystem Code
- Rotate in and out capability with PercentVBus and stop.
    - Up and down done at a constant speed.
    - Speed should be determined and tested (not arbitrary).
- Methods to access sensor information.
    - Limit switches.
    - Angle using the Through Bore Encoder.
•	Dashboard prints of sensor information.
•	Accurate sensors information.

#### Command
•	A command (or more) to move the arm up and down.
    - The commands should not use the limit switch for isFinished in this phase.

#### Robot Code
- Code in the robot class that creates the system and runs the command.
    - Attach commands to buttons such that:
        - Holding Dpad.Up raise the arm.
        - Holding Dpad.Down lower the arm.




## Swerve (Shalev)
### Implement the Subsystem 
There are some methods and abilities the swerve must have and some problems to fix. 

### Guidelines:
#### Subsystem Creation
- Create the swerve and the Module classes. Have relevant print for each system (velocities, position, angels etc).
- Swerve Module:
	- Each wheel has two motors- for driving and rotating use.
    - The position of the wheel (the angle in which it is) is determined by the absolute encoder.  You need to use it to set the “zero angle” of the wheels.

- Swerve:
    - This system must have a field relative option, to ease the driving. (The use of that already exists).
    - The swerve has a deviation while moving so we fixed it using PID for the robot’s angle. You can see the implementation of this in our code. Find a better way or recalibrate the PID’s constats to the movement will be better. 

#### Create Command
- You'll need a command to run your system with an Xbox controller.
- Make the driving with the Xbox a default command.  
- The driving direction should be based on the alliance in which you’re part of. Field relative determines what the Up in the joystick is, and what is down. In each match, you should base your forward direction by your alliance colour.
For example, if you’re the blue alliance and you need to give positive speed in order to move forward, you need to save that information and use it every time you use the joystick. That way, when the joystick says the speed is 0.5, it means you want to go backwards (it is negative to what you intend it to be), and therefore, you need to multiply the speed by (-1) for the robot to move backwards.
  
### Testing
- Try to drive in several directions and speeds. Make sure that the swerve is fast, drive strait when needed, and can spin easily while driving. 
- You can place cones and try drive between them. See how smooth is the driving. 

### Requirements:
#### Finished Subsystem Code
- Two ways to drive – field’s oriented and robot’s oriented. 
- Pose estimator / Odometry which will be updated constantly. 
- Better PID’s values for all motor controllers and PID controllers related to those systems.

#### Robot Code
- Code in the robot class that creates the system and runs the command as default command. 



## Climb (Daniel)
### Implement the Subsystem
- Include definitions for the motor controller and limit switches.
- Create methods for basic PercentVBus rotation of the motor, as well as a method for accessing the limit switches state.
- Add dashboard display of the limit switches state.
- Create a command to rotate the intake motor based on the XboxController.

### Guidelines:
#### Subsystem Creation
- Define all motor controllers and sensors used in the system.
- Construct these components in the constructor.
- Remember to configure the controller properly.
    - At the very least, reset to factory default.
- You will need to have the following set of methods:
    - A way to rotate the motor based on PercentVBus.
        - You should have constant speeds for up and down (up and down doesn't have to be the same).
    - A way to stop the motor rotation.
    - A way to access limit switches’ information.
- Remember to add print of sensors information to the dashboard.

#### Create Command
- You'll need a command to run your system with an Xbox controller.
- Create a command (or more) and attach them to buttons. At the very least, one button needs to raise the system up and one to lower the system down. 
- Configure it so that holding the buttons is required. This eliminates the need for `isFinished` for the moment.
- Don't use the limit switches in `isFinished` for now.

#### Testing
- Make sure the system moves as expected in both speed and direction and stops when reaching the limit switches.
- Check different speeds for moving up and down. Find optimal speeds and note them in your code. 
- Test the limit switches to make sure it works (use Shuffleboard to view its state).
- Move the system until it reaches one of the limit switches. See that in this state it can move in the opposite direction. Do this check for both limit switches and directions. 

### Requirements:
#### Finished Subsystem Code
- Rotate in and out capability with PercentVBus and stop.
    - Up and down done at a constant speed.
    - Speed should be determined and tested (not arbitrary).
- Methods to access sensor information.
    - limit switches.
- Dashboard prints of sensor information.
- Accurate sensor information.

#### Command
- A command (or more) to rotate the climb up and down.
    - The commands should not use the limit switches for `isFinished` in this phase.

#### Robot Code
- Code in the robot class that creates the system and runs the command.
    - Attach commands to buttons such that:
        - Holding `A` move climb up.
        - Holding `B` move climb down.
