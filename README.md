<div align="center">

# :soccer: INHA Defender
**Advanced Autonomous Agent for Humanoid Soccer**

[![ROS2](https://img.shields.io/badge/ROS2-Humble-3490dc.svg?style=for-the-badge&logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)
[![C++](https://img.shields.io/badge/C++-17-00599C.svg?style=for-the-badge&logo=c%2B%2B&logoColor=white)](https://en.cppreference.com/w/cpp/17)
[![BehaviorTree](https://img.shields.io/badge/BehaviorTree-V4-2ca02c.svg?style=for-the-badge)](https://www.behaviortree.dev/)
[![License](https://img.shields.io/badge/License-Apache_2.0-yellow.svg?style=for-the-badge)](LICENSE)

*Dynamic Decision Making • Tactical Positioning • Strategic Passing Decisions*

---
</div>

## Defender Vision
**"To create a soccer-playing intelligence that doesn't just calculate, but *understands* the flow of the game."**

The **INHA Defender** is designed to bridge the gap between rigid robotic control and dynamic human intuition. By leveraging hierarchical behavior trees and advanced motion planning, our agent demonstrates adaptive gameplay—switching seamlessly between defensive clearing, tactical off-the-ball movement, and line-breaking passing.

---

## Key Feature

### **Hyper-Modular Architecture**
We separate **Strategic Intent** from **Mechanical Execution** using a novel **Parameter-Injection Pattern**. This allows the robot to adapt its personality in real-time without recompiling the core logic.

### 1. Strategy Layer (The Director)
Defines the high-level intent based on the match context.
* **Role**: Analyzes the environment and sets the global "Mode."
* **Example**: `Score < Opponent` & `Time Remaining < 2min` → Switches to **`ALL_OUT_ATTACK`**.

### 2. Tactics Layer (The Tuner)
Translates strategy into specific constraints via the **BehaviorTree Blackboard**.
* **Role**: Injects parameters (speed, aggression, thresholds) instead of hard-coding behaviors.
* **Feature**: **1:N Mapping** (Context-Aware Selection)
    * Same Strategy can lead to different Tactics based on `ScoreDiff`, `BallPosition`, etc.
    * **Example**:
      * **`OFFENSIVE` Strategy**:
        * *Losing by 3+ goals*: `TOTAL_ASSAULT` (Risky Attack)
        * *Ball in own half*: `COUNTER_ATTACK` (Fast Break)
        * *Otherwise*: `PRESSING` (Standard Pressure)
      * **`DEFENSIVE` Strategy**:
        * *Winning by 3+ goals*: `TEMPO_CONTROL` (Possession Game)
        * *Ball near own goal*: `DEEP_DEFENSE` (Park the Bus)
        * *Otherwise*: `LINE_DEFENSE` (Standard Defense)

### 3. Execution Layer (The Engine)
The robust `DefenderDecide` node and leaf nodes consume these parameters to perform actions.
* **Role**: Executes the "How" based on the "What" provided by the Tactics layer.
* **Example**: The `Chase` node reads `speed_limit = 1.0` and triggers a max-speed sprint, while `DefenderDecide` uses the loose `kick_threshold` to shoot at the first opportunity.


> **💡 The Benefit**
> You can completely overhaul the robot's playstyle—from a conservative defender to a hyper-aggressive striker—just by tweaking a few numbers in the Tactics layer, with **zero risk** of breaking the core movement logic.

> #### **📂 Proof of Modularity: Code Structure**
> Our source tree is explicitly organized to enforce this architectural separation:
> 
> * 📂 **[`src/brain/src/`](src/brain/src)**
>   * 📂 **[`strategy/`](src/brain/src/strategy)** — *(Layer 1: Strategy Director)*
>     * 📄 [`strategy_director.cpp`](src/brain/src/strategy/strategy_director.cpp)
>     * 📄 [`game_state_manager.cpp`](src/brain/src/strategy/game_state_manager.cpp)
>     * 📄 [`strategy_nodes.cpp`](src/brain/src/strategy/strategy_nodes.cpp)
>   * 📂 **[`tactics/`](src/brain/src/tactics)** — *(Layer 2: Tactics / Tuners)*
>     * 📄 [`tactic_selector.cpp`](src/brain/src/tactics/tactic_selector.cpp)
>     * 📄 [`tactics_definitions.cpp`](src/brain/src/tactics/tactics_definitions.cpp)
>     * 📄 [`tactics_nodes.cpp`](src/brain/src/tactics/tactics_nodes.cpp)
>   * ⚙️ **Layer 3: Execution Engines** — *(Consumers)*
>     * 📄 [`decision_role.cpp`](src/brain/src/decision_role.cpp) : **Main Decision Logic**
>     * 📄 [`offtheball.cpp`](src/brain/src/offtheball.cpp)
>     * 📄 [`chase.cpp`](src/brain/src/chase.cpp)
>     * *... (kick, adjust, etc.)*

---

## Defender Behavior Tree Overview


<img width="3509" height="1492" alt="Defender" src="https://github.com/user-attachments/assets/8740f085-cc16-4508-a4e4-d47cc7c5d9a6" />



---

## Contribution
This project contributes to the field of humanoid robotics by:
1.  **Demonstrating Robust Autonomy**: Showing how behavior trees can handle the chaotic environment of a soccer match.
2.  **Implementing Human-inspired Motion**: Proving that curvilinear paths are superior to linear point-to-point navigation for bipedal robots.
3.  **Open Source Innovation**: Providing a modular, extensible C++ framework for future researchers in the RoboCup domain.

---

<div align="center">
    <b>Built with by INHA United</b><br>
    <i>Pushing the boundaries of Autonomous Soccer</i>
</div>
