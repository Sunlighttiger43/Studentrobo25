# Automated PID Tuning Strategy

This document describes a strategy for **automated PID tuning** for the physical robot’s key movement functions in `olaves_robot.py`:
- `_turn_to_marker`
- `move_to_marker`
- `_move_to_marker_finalisation`
- `move_pos`

The goal is to have a **self-improving** system that:
1. **Remembers** previous PID configurations and their performance (stored in a JSON file).
2. **Proposes** new candidate PID gains using a simple exploration strategy (akin to a reinforcement-learning approach).
3. **Tests** the new gains under controlled conditions (in simulation or on the real robot).
4. **Evaluates** performance metrics (time-to-target, overshoot, error) to derive a "score."
5. **Updates** the JSON store with the best-performing PID gains.

In particular, we also want to allow **two PID controllers** to be tuned at once (e.g., distance PID and angle PID in `move_to_marker`). This allows the system to improve both controllers simultaneously.

Below is an overview of the process and the essential code scaffolding.

---

## 1. Overall Tuning Flow

1. **Startup Check**  
   - On robot startup (in `robot.py`), load a JSON file containing historical tuning data.  
   - If no file is found, create default entries.

2. **PID Gains Proposal**  
   - For each PID controller to be tuned (e.g., distance PID, angle PID), propose new gains by exploring around the best known gains or by random sampling within a safe range.  
   - E.g., `p_new = p_best ± random_small_change`, same for `i` and `d`.

3. **Assign Proposed Gains**  
   - Write these proposed gains into the dictionary the robot uses (e.g., `self.pid_tunings[...]` in `olaves_robot.py`).

4. **Run a Test Routine**  
   - The test routine attempts a known scenario (e.g., turning to a marker, or moving to a known pose) that exercises the relevant PID.  
   - The code tracks performance metrics—time to reach tolerance, overshoot, final steady-state error.

5. **Evaluate Performance**  
   - Summarise the run with a performance "score." Lower is better. For example:  
     \[
       \text{score} = \alpha \times (\text{time\_to\_reach\_tolerance}) + \beta \times (\text{max\_overshoot}) + \gamma \times (\text{final\_error})
     \]
   - The weighting factors \(\alpha, \beta, \gamma\) can be tweaked for your application.

6. **Store Results and Update**  
   - If the new run's score is better than the best known score, update the stored "best gains."  
   - Keep a small log (e.g., a few recent runs) so that future starts can see how exploration steps changed performance.

7. **Re-Use**  
   - Next startup, the system loads these best gains, then tries a slightly different set again, continuing to refine.

This approach can be extended to simultaneously tune **two PID controllers** by:
- Proposing changes to both sets of PID gains (distance + angle).
- Running a combined test scenario (like `move_to_marker` which uses both).
- Evaluating the combined performance.  
This single “score” can reflect the synergy of both controllers.

---

## 2. Data Storage Format

You might keep a `pid_tuning_history.json` with structure:
```json
{
  "move_to_marker": {
    "best_gains_dist": { "p": 0.1, "i": 0.01, "d": 0.0 },
    "best_gains_angle": { "p": 0.3, "i": 0.05, "d": 0.2 },
    "best_score": 12.3,
    "history": [
      {
        "gains_dist": { "p": 0.08, "i": 0.01, "d": 0.0 },
        "gains_angle": { "p": 0.25, "i": 0.05, "d": 0.2 },
        "score": 15.2
      },
      {
        "gains_dist": { "p": 0.1, "i": 0.01, "d": 0.0 },
        "gains_angle": { "p": 0.3, "i": 0.05, "d": 0.2 },
        "score": 12.3
      }
    ]
  },
  ...
}
```
Storing multiple sections for _turn_to_marker, _move_to_marker_finalisation, etc.

---

## 3. Proposed Tuning Manager
We introduce a PIDTuningManager class to handle:

    Loading old data.
    Proposing new gains.
    Storing performance results.

Basic Idea

    Load: If no best data is found, use defaults.
    Propose: For each PID, do something like:

```python
best_gains_angle["p"] += random.uniform(-0.01, 0.01)
...
```

Test: Run a small routine that calls move_to_marker or move_pos.
Record: Build a score from the test results, compare with best, store accordingly.

