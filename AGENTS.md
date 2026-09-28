# This robot package

Stack: ROS 2 + C++.
Audience: a developer who is still learning both.

## Code comments (required)

When you write or change C++:

1. Every new public class/node file starts with a 4–8 line comment:
   - what this node/component does on the robot
   - what it subscribes/publishes/serves
   - what it assumes already exists (other nodes, TF, params)
2. Every callback / timer / service handler says:
   - what event triggers it
   - what it may block or must not block
   - which data it reads/writes
3. Do not leave "magic" QoS, frame names, or executor choices uncommented.

## Explanation requests

If the user asks "explain this" or "why this code":
concept first, then the C++/ROS 2 mechanism, then the local code.
Do not start with the implementation.
