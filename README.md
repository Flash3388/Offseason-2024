# Offseason 2024

## Desired Goals

### Quick and Accurate Control Loops
- For all control loops (mostly Arm and Shooter).
- Quick and accurate set-point settling.
- Based on SparkMAX-integrated PIDF for all loops (not necessary for the Arm, but might be possible). 
- Possible usage of SmartMotion (check this option for the arm).

### Proper System Limits
- Use software or hardware limits for Arm and Climb systems.
- Insure they prevent damaging the robot. Be careful!!!

### Capable Auto-Firing
- Usage of Interpolation (ask for help), accurate Arm and Shooter control and target acquisition.
- Be capable of looking-on and firing notes at targets from different orientations and ranges (using vision).
- As quick a process as possible (as soon as a note is collected, be ready to shoot at an instant). Think how you will do it.
   - several systems must reach set points to fire
   - during the season, there was an attempt to use vision to align to shooter - it produced bad offsets and the set point wasn't reached
   - during the season, on occaison the shooter and arm didn't reach the set points, reason unknown
   - need to spend a lot of time analyzing and improving the system operations to acheive this
- Control all relevant systems automatically, no need for additional operator input.
- Odometery or Vision based targeting
- Possibility for firing without human operator approval (?)
- with the press of a button, do everything needed for shooting (or shoot automatically when close enough)
- try keeping robot aligned on the target when ready for shoot
    - using it as a like-field relative, just not oriented by field but via target
    - old implementation controlled the rotation component to keep alignment
- move to amp position automatically 

### Accurate Field Odometery
- Usage of Pigeon and encoders for basic Odometery. Show the robot on the field live. Make sure the robot knows it exact location and how to calculate and consider its movement. 
- Integration of Vision for error correction and initial position configuration (using vision).
- Verify (using tape measure and such on a simulated field) that calculations of positioning and offsets is accurate

### Auto-Collection (using vision - probably) 
- Detect and reach notes on the floor.
- Collect them once reached.
- Likely vision-based note locating.

### PathPlanner
- Usage of PathPlanner to plan and execute complex paths for autonomous mode.
- Integrated with event markers.
- Quick, accurate and smooth motion.
- Possibility of on-the-fly paths in teleop.

### Power Consumption 
- Make sure systems don't consume to much power. You can do it by limit the amount of current or voltage they consume. You can consult with the mentors about how much to limit each system. 
- Make sure we can fully run the robot and its systems for the duration of a game on a battery (2.5 minutes).
- When shooting (with shooter and arm) - analyze power consumption: could it cause problems? are they consuming too much power

### Auto-Climb
- try to move the robot to the chain automatically and position it ready to climb. Use odometery
- maybe add a camera to allow driver to orient to the chain

### Automatic Operations
- Use as many as possible commands groups. Reduce the use of Xbox’s buttons to a minimum. For example, one button to aim the robot orientation to the target, raise the Arm to the right angle and shoot – all in one button. 
- Think how you can make the robot more automatic. It can help you earn prizes and help the driver while playing.
- try to reach a point where there is only one driver, no need for another

### For Competition
- Prepare checklist for pre and post game to check robot as operational
- Perhaps prepare a set of automatic tests? (how?)
- Check bandwidth problems with FMS and limelight
- Design Shuffleboard view

## Notes on System/Operations

### Drive
- use field relative for most of the match
    - orient depending on the alliance
- pose estimator mixing odometery and vision
- for rotation: provide more power then x-y drive to be able to actually rotate
- swerve has a weird tendency to rotate a bit off-axis, can use active PID solution to compensate  

### Arm
- The arm is a difficult system, really heavy and big
- properly-tuner PID could help stabilize it, but it requires a lot of works from the motors
- both motors must function together to keep it operational
- using Feed-Forward would be essential to keeping it steady
- using SparkMax integrated PIDF could help
- slow arm speed down as it reaches its limits to prevent it from causing damage. Perhaps introduce software limits with a buffer zone from the hardware limits
- in testing: make sure arm is steady enough during motion
- calibrate zero angle for when using through-bore
- use through-bore via SparkMAX
- consider situations where the absolute encoder disconnects: how to recognize and handle?

### Shooter
- use Feed-Forward for tuning
- use integrate PIDF for SparkMax
- present min and max speeds for shooting
    - at low speeds, the note loses velocity quickly and tumbles down
- shooting to amp requires close proximity, determine specfic speed
- shooting to speaker can use variable speeds   

### Intake
- operate motors in break mode to keep note in place
- use both proximity sensors to recognize note
- find right output for quick collection and shooting
- investigate problems with proximity sensors disconnecting
- build team: how to make sure that sensors stay in place
- difficult to collect manually:
    - field relative control isn't good for this
    - hard to orient to note
    - difficult to catch with intake

# Climb
- don't operate if the arm isn't raised or else the hook will hit electronics (add a software limit)
- at the start of the match, when arm is raised enough, fix climb back to its open position

### Auto-Firing
- Consider auto-firing interpolation by arm angle or by shooter speed (or both)
- Use proper interpolation methods (e.g. Langrage)

## Part 1

### Intake (Stav)
#### Implement the Subsystem
- Include definitions for the motor controller and proximity sensors switches.
- Create methods for basic PercentVBus rotation of the motor, as well as a method for accessing the proximity sensors switches state.
- Add dashboard display of the proximity sensors switches state.
- Create a command to rotate the intake motor based on the XboxController.

#### Guidelines:
##### Subsystem Creation
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

##### Create Command
- You'll need a command to run your system with an Xbox controller.
- Create a command (or more) and attach them to buttons. At the very least, one button needs to pull note in and one out.
- Configure it so that holding the buttons is required. This eliminates the need for `isFinished` for the moment.
- Don't use the proximity sensors switches in `isFinished` for now.

##### Testing
- Make sure the system moves as expected in both speed and direction.
- Check different speeds for note in and note out. Find optimal speeds and note them in your code.
- Try inserting the note from different positions and directions to make sure the note is collected well.
- Test the proximity sensors switches to make sure it works (use Shuffleboard to view its state).
- Push in a note and see when the proximity sensors switches detects the note.
- Pull the note out and see when the proximity sensors switches no longer detect the note.

#### Requirements:
##### Finished Subsystem Code
- Rotate in and out capability with PercentVBus and stop.
    - In and out done at a constant speed.
    - Speed should be determined and tested (not arbitrary).
- Methods to access sensor information.
    - Proximity sensors switches.
- Dashboard prints of sensor information.
- Accurate sensor information.

##### Command
- A command (or more) to rotate the intake in and out.
    - The commands should not use the proximity sensors switches for `isFinished` in this phase.

##### Robot Code
- Code in the robot class that creates the system and runs the command.
    - Attach commands to buttons such that:
        - Holding `RB` pulls note in.
        - Holding `LB` pushes note out.



### Arm (Niv)
#### Implement the Subsystem
- Include definitions for the motor controller and limit switch.
- Create methods for basic PercentVBus rotation of the motor, as well as a method for accessing the limit switch state.
- Add dashboard display of the limit switch state.
- Create a command to raise and lower the Arm based on the XboxController.

#### Guidelines:
##### Subsystem Creation
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
##### Create Command
- You'll need a command to run your system with an Xbox controller.
- Create a command (or more) and attach them to buttons. At the very least, one button needs to raise the arm, and one should lower it.
- Configure it so that holding the buttons is required. This eliminates the need for isFinished for the moment.
- Don't use the limit switch in isFinished for now.
  
#### Testing
- Make sure the system moves as expected in both speeds and directions.
- Check different speeds for razing and lowering the arm. Find optimal speeds and note them in your code.
- Test the limit switch to make sure it works (use Shuffleboard to view its state).

#### Requirements:
##### Finished Subsystem Code
- Rotate in and out capability with PercentVBus and stop.
    - Up and down done at a constant speed.
    - Speed should be determined and tested (not arbitrary).
- Methods to access sensor information.
    - Limit switches.
    - Angle using the Through Bore Encoder.
•	Dashboard prints of sensor information.
•	Accurate sensors information.

##### Command
•	A command (or more) to move the arm up and down.
    - The commands should not use the limit switch for isFinished in this phase.

##### Robot Code
- Code in the robot class that creates the system and runs the command.
    - Attach commands to buttons such that:
        - Holding Dpad.Up raise the arm.
        - Holding Dpad.Down lower the arm.




### Shooter (Talya)
#### Implement the Subsystem
- Define both motors for shooter and encoder.
- Define your motor's built in PIDController
- Configure the motors and PID.
	- Factory reset both motors.
 	- Set the output limit to 1 and -1.
  	- Reset the PID.


#### Guidelines:
##### Subsystem Creation
- Create basic methods such as:
	- Rotating the motors 
- Create the shooter, create relevant print and method for the shooter.
	- Create a method that returns the velocity of the motor.
	- Create a method that returns if the velocity got to the required RPM.

##### Create Command
- Create a command that rotates the motor to a selected velocity.
	- It needs to be ran on the click of a button on a Xbox Controller.

##### After Basic Creation
 - Look for methods of rotating the wheel since it's a flywheel.
 	- Fly wheel means it just rotates freely it just needs to get to it's speed.
 	- Can you use FeedForward alone without PID?
- Change the command you created to use PID or the method you decided to go with.

#### Testing
- Using the Xbox Controller check if the wheel spins to the wanted velocity.

#### Requirements:
##### Finished Subsystem Code
- Able to get the shooter to get to wanted velocity in a quick time.
- Tested for best methods for getting the motor to it's wanted velocity fast.

##### Robot Code
- Add the commands and their corresponding buttons to use them:
	- Click X to run get to velocity command.




### Swerve (Shalev)
#### Implement the Subsystem 
There are some methods and abilities the swerve must have and some problems to fix. 

#### Guidelines:
##### Subsystem Creation
- Create the swerve and the Module classes. Have relevant print for each system (velocities, position, angels etc).
- Swerve Module:
	- Each wheel has two motors- for driving and rotating use.
    - The position of the wheel (the angle in which it is) is determined by the absolute encoder.  You need to use it to set the “zero angle” of the wheels.

- Swerve:
    - This system must have a field relative option, to ease the driving. (The use of that already exists).
    - The swerve has a deviation while moving so we fixed it using PID for the robot’s angle. You can see the implementation of this in our code. Find a better way or recalibrate the PID’s constats to the movement will be better. 

##### Create Command
- You'll need a command to run your system with an Xbox controller.
- Make the driving with the Xbox a default command.  
- The driving direction should be based on the alliance in which you’re part of. Field relative determines what the Up in the joystick is, and what is down. In each match, you should base your forward direction by your alliance colour.
For example, if you’re the blue alliance and you need to give positive speed in order to move forward, you need to save that information and use it every time you use the joystick. That way, when the joystick says the speed is 0.5, it means you want to go backwards (it is negative to what you intend it to be), and therefore, you need to multiply the speed by (-1) for the robot to move backwards.
  
#### Testing
- Try to drive in several directions and speeds. Make sure that the swerve is fast, drive strait when needed, and can spin easily while driving. 
- You can place cones and try drive between them. See how smooth is the driving. 

#### Requirements:
##### Finished Subsystem Code
- Two ways to drive – field’s oriented and robot’s oriented. 
- Pose estimator / Odometry which will be updated constantly. 
- Better PID’s values for all motor controllers and PID controllers related to those systems.

##### Robot Code
- Code in the robot class that creates the system and runs the command as default command. 



### Climb (Danielle)
#### Implement the Subsystem
- Include definitions for the motor controller and limit switches.
- Create methods for basic PercentVBus rotation of the motor, as well as a method for accessing the limit switches state.
- Add dashboard display of the limit switches state.
- Create a command to rotate the intake motor based on the XboxController.

#### Guidelines:
##### Subsystem Creation
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

##### Create Command
- You'll need a command to run your system with an Xbox controller.
- Create a command (or more) and attach them to buttons. At the very least, one button needs to raise the system up and one to lower the system down. 
- Configure it so that holding the buttons is required. This eliminates the need for `isFinished` for the moment.
- Don't use the limit switches in `isFinished` for now.

##### Testing
- Make sure the system moves as expected in both speed and direction and stops when reaching the limit switches.
- Check different speeds for moving up and down. Find optimal speeds and note them in your code. 
- Test the limit switches to make sure it works (use Shuffleboard to view its state).
- Move the system until it reaches one of the limit switches. See that in this state it can move in the opposite direction. Do this check for both limit switches and directions. 

#### Requirements:
##### Finished Subsystem Code
- Rotate in and out capability with PercentVBus and stop.
    - Up and down done at a constant speed.
    - Speed should be determined and tested (not arbitrary).
- Methods to access sensor information.
    - limit switches.
- Dashboard prints of sensor information.
- Accurate sensor information.

##### Command
- A command (or more) to rotate the climb up and down.
    - The commands should not use the limit switches for `isFinished` in this phase.

##### Robot Code
- Code in the robot class that creates the system and runs the command.
    - Attach commands to buttons such that:
        - Holding `A` move climb up.
        - Holding `B` move climb down.
     
## Part 1.5

We have several things to do to update, test and advance new features

- Intake
    - Try to align the sensors with tape to keep them in place temporarily
    - Wait for build team to construct strong placements for them
    - Go through code and look for problems
 
- Arm
    - Test Through-Bore interfacing via SparkMax
        - Configure it to our wanted angle parameters
    - Introduce Software limits via Through-bore
    - Implement PID control for Arm
        - Idealy, use Spark PIDF with Through-Bore connected to it
        - See how to integrate smarter Feed-Forward
        - Consider Smart Motion   

- Climb
    - Go through code and look for problems
    - Check about motion directions and limit switches, as it was reported that they were inverted
        - make sure limit switches stop motions actually

- Shooter
    - Look into new sensor to detect note passing through
        - Find a sensor, solder it, place it on shooter and test it
    - Go through code and look for problems
    - Make sure shooter can reach any RPM quickly, take a look at graphes and such

- Swerve
    - Check pigeon functionality
    - Check field odometery
    - Check/implement field oriented drive
    - Look into stabilizing rotation with PID 

- Vision
    - Noam and Yahav will train
    - Learn about how to connect and configure limelight settings
    - Produce results in steps
        - AprilTag Detection
            - configure to detect AprilTag
            - explore possible settings
            - write code to read AprilTag information
        - Robot Pose extrapolation
            - Configure Limelight to provide Robot Pose
            - Read Robot pose from limelight and update odometery    
