1. set Initial gains
Start with P-only control: Set I and D to zero
Increase P (Proportional Gain) gradually util the system oscillates around the target. if it oscillates too much, reduce P slightly
2. Tune the Integral Gain (I)
Introduce a small I value to reduce steady-state error
Increase I slowly until the system reaches the target with minimal offset
If the system becomes sluggish or oscillates excessively, reduce I.
3. Tune the Derivative Gain (D)
Introduce D to reduce overshoot and dampen oscillations
Increase D gradually until oscillations are controlled without excessive lag
Too much D can make the system too sensitive to noise
4. Test Different Movements
Straight-line motion: Ensure steady movement without oscillations
Angle gain (steering correction): Adjust the controller so that wheel angles correct smoothly
Turning response: Ensure the system responds properly to turning commands without overshooting or lagging
5. Fine tuning on surfaces
Test on different surfaces to ensure robustness
Adjust for varying speeds to maintain stability
Check the response to sudden changes