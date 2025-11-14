# ME449 Assignment 4
*Author: Conor Hayes*

## Introduction
### Contents
1. PDF file (this file) -- see sections below
2. commented code is contained in `code` directory
3. output CSV's are in the top-level submission zip, and also are placed in `code/output/*` when the code is run
5. videos are in the top-level submission zip

### Running the code
#### using `uv` (no virtualenv needed)
```
uv sync
uv run assignment_4.py
```

#### using `pip` (recommended: use virtualenv)
```
pip install -r requirements.txt
python assignment_4.py
```

## Part 1
For part A, `dt=0.001` was chosen, which gave a reasonable energy-preserving
trajectory as evaluated visually in CopelliaSim.

For part B, `dt=0.02` was chosen, causing the robot arm's trajectory to chaotically
increase energy as time goes on, once more evaluated visually.

In order to plot energy over time, we could calculate energy for each timestep
inside of `puppet()` as follows (all variables on the right hand side of the
equation are already available at runtime inside the function as implemented):

$KE = .5 * \dot{\theta}^T M(\theta) \dot{\theta}$

Then we can save this energy in an array to be returned at the end of `puppet()` execution.

## Part 2
For part A, `damping=1.0` was selected, and for part B, `damping=-0.01` was selected.

Very large constant damping values produce overflow values in numpy
for this implementation, but in general, large damping values with
insufficiently small values of `dt` will produce
oscillations in simulation; this is because a small velocity will produce a 
sufficiently large acceleration to flip the sign of the joint velocity over
the course of `dt`, resulting in a flip-flopping effect that may
escalate out of control.

This is only an artifact of simulation, however, because in continuous time
the damping force can only reduce the velocity to 0, not reverse the sign. 
Similarly, if `dt` is shrunk, the simulation can handle larger damping values
without such oscillations.

## Part 3
For part A:
```
stiffness=2.0
damping=0.001
```
At this low damping value, I'd expect the system's total energy to decrease
very slightly over time. 

For part B:
```
stiffness=100.0
damping=4.0
```
At this higher stiffness + damping value, I'd expect:
- system total energy decreases over time
- visually, the arm is pulled tightly to the string and gradually stops oscillating.

We see the above behavior in the videos attached.

## Part 4
See video. constants from 3B were used again.

