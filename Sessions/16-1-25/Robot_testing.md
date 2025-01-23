PID controls done - just need to change polarity and then wheels willt turn in the right direction
Logic for positioning system has been nailed down
Still need to do the Movement challenge - mechanical challenge done (and suprisingly accepted - ingenuity)

What needs to be done:
- Implement move_pos()
- Need to actually code positioning of the robot
- Need to code pick up box code
- Need to actually code hunting state and what the robot will do (ie the main loop of the program)
    - Methods involved:
    - Factor in Number of pallets into intensity calculation
    - Get_number_of_pallets on high rises
    - Modify the decision statements to reflect any changes in the intensity calculation (in the run() method in states)
    - Change the implementation of searching for markers in class Hunting to account for the fact our robot could be in any one of the 4 zones
    - Add logic to handle how the marker is dealt with onoce approached
    - Add interrupt logic to the main loop.
 

Lots of this will need to be done outside of the session with testing to be done within the session
Will send out announcements as to what each person should do
