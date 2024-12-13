# Intro
This pre-session notes thing will be something I do before every session (to the best of my ability and memory). It will act as a guide for what needs to be done during and after the session, acting as a guide so that we can stay on track, and everyone has something to do. As already stated, this is a *guide* and not prescriptive; of course, you may decide to work on something else, but use your best judgement to decide if something needs extra hands. When in doubt, ask Jawad.

# Coding Team
The Coding Team is in the struggle zone right now. We have a lot of things that we need to work on simultaneously. I won't be in this session, but if you have any queries, you may message me or ask Jawad (probably your best course of action). 

Also, everyone needs to understand PID control, so ask Jawad or JSav for a crash course.

## Positioning and Localisation
Our robot has no sense of direction and is about as great at navigating as a pigeon without the Earth's magnetic field; we need to amend that. Whilst there is code currently for dealing with this issue—localising (getting the robot's pose) and positioning (setting the robot's pose to some value)—it is currently bugging. I have tried relentlessly to get the code to work, but I think I am greatly overcomplicating things for student robotics, so I will leave this task to you.

You will need to:
- Develop an adequate mathematical foundation behind the algorithm to get the pose of the robot
    - We will take the origin to be the corner we start in
    - We want to return a tuple (r_x, r_y, r_theta) corresponding to the obvious values
- With the mathematical foundation, you need to implement the algorithm; this should hopefully be the easy part
- Test and debug the code with rigour
- If the get_pose code is done, then you can work on the set_pose code
    - You will need to learn and understand the maths behind PID control (Ask Jawad/JSav for this)
    - Use this to inform the development of a naive algorithm that moves the robot from one pose to another, assuming there are no obstacles in the way
        - If this is too trivial, you can consider implementing path planning to avoid obstacles (namely the highrises)
- If you get all of the above done in this session, I will be infinitely impressed
- P.S. There is an ultrasound sensor which may prove helpful

## States
I am not sure about the state of the states (heh...). If I recall correctly, though, quite decent progress has been made, so keep it up.

You will need to:
- Check the implementation of the hunting state
    - If it is fine and the positioning code is not complete to test, then do not worry about it for now
    - If I recall correctly, it was mostly pseudocode; you should try considering the actual implementation
- Implement and test the knocking state
    - I forget who, but someone has already created some skeleton code for the knocking state; whoever this can take charge of testing and implementing this code into the rest of the program loop
 If the positioning code is not complete to test, then do not worry about it for now
- Design function in OlavesRobot class to pick up a marker
    - You would need to design a function to move the robot to a marker
        - There was a function like this, but completely untested, so you may need to understand PID control to debug or rewrite the code entirely
        - Then add this function to another function which will use the suction to pick up the marker
    - You will also need to use the same tech to develop a deposit function
- If the positioning stuff is complete, or if you do not need it to test the states, then test the functionality of the states

## Main Program Loop
I also don't know where we got with this, but from what I see, there's nothing there.

You will need to:
- See the "Meeting Minutes/Coding Team/2-12-24.md" for all the notes on this

## Movement Challenge
While everything above can be done using the simulator, we still need to get the physical robot to do stuff and look ahead to the next challenge deadline. We will do the movement challenge for this challenge, where you will need to move in a triangle.

You will need to:
- Plan out a strategy for getting the robot to do this (it would be incredibly wise to mess around with robot movement to get a feel for things and to help some ideas brew)
- Implement the strategy and test it
- If it works, film it
- I cannot give much more advice than that, other than to read the rules stringently. If it helps, there is an ultrasound distance sensor that you can use.

# Design Team
Honestly, you guys seem organised within yourselves. I'll leave it to you; just don't forget to keep everyone updated with what's going on.

Oh, and if you can get some components designed by the mid-to-late Winter Term holiday, that would be huge, as JSav would be able to get a headstart on ordering stuff to give the manufacturing team something to do. But don't rush yourselves; quality > speed.

# Manufacturing Team
Similar to the design team, one thing I would say, however, is to double-check the wiring and that you all understand how it works. There are still a few components that we need to attach to the test robot, so use these as good practice for the actual competition and also allow the coding team to actually make progress on the movement challenge.

If you are on this team, lack things to do, and have expertise in the design or coding team, consider lending your talents to help out.



Thanks, and good luck with the session today, guys.