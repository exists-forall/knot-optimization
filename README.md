# knot-optimization
A UC Berkeley research project to generate 3D-printed models of topological knots

## Purpose

This project's goal is to automatically generate instructions for assembling 3D-printed elbow-shaped components into physical models of [mathematical knots](https://en.wikipedia.org/wiki/Knot_(mathematics)).  This task is constrained by the fact that the components may only lock together at certain discrete angles, and that the final knot should be as symmetric as possible.

![Trefoil](https://imgur.com/lNEg5lY.png)

## To Run

Required Software:
1. Install [Rust](https://www.rust-lang.org/tools/install).
2. Pull this repository.

Designating Your Own Knot:
1. Create a [bspline](https://en.wikipedia.org/wiki/B-spline) for the knot you'd like to model. Your knot should be symmetrical, and your bspline should only map
one segment of the knot. For example, if you want to create a trefoil knot that
is comprised of three symmetrical components, your bspline should only contain
one third of the trefoil knot.
2. Head over to ~/knot_optimization/knot/src/geometries/custom_spline.rs and input your bspline, following the comments. You'll need the bspline's degree and control points. If you have parameters, there are comments for how to use those.
3. Go into ~/knot_optimization/knot/src/bin/visualize_curve.rs and input the knot's symmetry accordingly.
4. To view the curve, navigate into ~/knot-optimization/knot, and run the command
``cargo run --bin visualize_curve --release``
5. Using this visualization, adjust the value of scale to increase or decrease the size of the knot
as you see fit. Keep the following in mind:
    1. Remember that we don't want the pieces to be too close together. The radius of the pieces in the
    visualization is the same as the radius of the joints; if the pieces are forced to be too close they
    may vibrate in the visualization, which is a sign that you'll need to increase the scale.
    2. Remember that our joints only have one bending angle, 120&deg;, so any curves in the knot should be
    relatively smooth. If your visualization has tight turns, it may be another sign to increase the scale.
    3. Making knots at a larger scale comes at a clear tradeoff: the larger the knot is, the more pieces
    it will require to be assembled. This also comes at an efficiency cost, but for reasonably-sized
    knots this is relatively negligible.
6. If you'd like to interact with your knot, you can run the command
``cargo run --bin continuous_optimization --release`` from ~/knot-optimization/knot.
This will display your knot in a screen, constructed with our joints but with no
locking angle constraint - this means that your components will behave as if they
have circular cross-section. Below is a list of commands:
    1. You can use "Return" to turn on locking weights, and "\" to turn them off.
    2. You can use "Right" and "Left" to increase and decrease the locking weights,
    respectively. This will have no effect if the locking weight is turned off.
    3. You can use "Space" to print out the current approximation of each joint's
    "angle" on the segment - how many turns you would have to make on each joint.
    Note that an angle of 0 is described to be the case where the joints "don't
    turn." One way to visualize this would be to say that a full chain of joints
    with angle 0 would make a circle.
    4. The keys "M" and "N" turn on and off (respectively) a force that pushes non-adjacent joints away from one another. "K" and "J" can be used to increase and decrease (respectively) that force. We suggest leaving this on.
    5. You can select joints in your segment and rotate them. Note that doing so rotates the corresponding joints in the symmetric segments. Selecting joints can be done with the "Up" and "Down" keys, and turns can be made with the "Equals" and "Minus" keys.
