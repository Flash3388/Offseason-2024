# Offseason 2024

## Intake

### Implement the Subsystem
- Include definitions for the motor controller and limit switch.
- Create methods for basic PercentVBus rotation of the motor, as well as a method for accessing the limit switch state.
- Add dashboard display of the limit switch state.
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
    - A way to access limit switch information.
- Remember to add print of sensors information to the dashboard.

#### Create Command
- You'll need a command to run your system with an Xbox controller.
- Create a command (or more) and attach them to buttons. At the very least, one button needs to pull note in and one out.
- Configure it so that holding the buttons is required. This eliminates the need for `isFinished` for the moment.
- Don't use the limit switch in `isFinished` for now.

#### Testing
- Make sure the system moves as expected in both speed and direction.
- Check different speeds for note in and note out. Find optimal speeds and note them in your code.
- Try inserting the note from different positions and directions to make sure the note is collected well.
- Test the limit switch to make sure it works (use Shuffleboard to view its state).
- Push in a note and see when the limit switch detects the note.
- Pull the note out and see when the limit switch no longer detects the note.

### Requirements:

#### Finished Subsystem Code
- Rotate in and out capability with PercentVBus and stop.
    - In and out done at a constant speed.
    - Speed should be determined and tested (not arbitrary).
- Methods to access sensor information.
    - Limit switch.
- Dashboard prints of sensor information.
- Accurate sensor information.

#### Command
- A command (or more) to rotate the intake in and out.
    - The commands should not use the limit switch for `isFinished` in this phase.

#### Robot Code
- Code in the robot class that creates the system and runs the command.
    - Attach commands to buttons such that:
        - Holding `RB` pulls note in.
        - Holding `LB` pushes note out.
