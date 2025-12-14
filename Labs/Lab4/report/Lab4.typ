/**************   Report Template   **************/
#import "@preview/codly:1.3.0": *
#import "@preview/codly-languages:0.1.1": *

#show: codly-init.with()

#let table-continued = state("table-continued", false)

#let report-template(
  title: "",
  course: "",
  author-name: "",
  author-id: "",
  group: "",
  date: "",
  body
) = {
  set document(title: title, author: author-name)

  set page(
    paper: "a4",
    margin: 2.5cm,
    header: context {
      if counter(page).get().first() > 1 {
        set text(size: 11pt, font: "Times New Roman")
        grid(
          columns: (1fr, auto, 1fr),
          align: (left, center, right),
          [#title],
          [#author-name (#author-id)],
          [#group]
        )
        v(-10pt)
        line(length: 100%, stroke: 0.6pt)
      }
    },

    footer: context {
      align(center)[
        #counter(page).display("1")
      ]
    }
  )

  set text(
    font: "Times New Roman", 
    size: 12pt,
    lang: "en",
    region: "GB"
  )

  set math.mat(delim: "[")
  set math.vec(delim: "[")
  set math.equation(numbering: "(1)")

  show ref: it => {
    let el = it.element
    if el != none and el.func() == math.equation {
      let count = counter(math.equation).at(el.location())      

      let num = numbering(el.numbering, ..count)

      set text(fill: rgb("0000FF")) 
      link(el.location(), num)
    } else {
      it
    }
  }

  show link: set text(fill: rgb("0000FF"))

  set par(
    justify: true, 
    first-line-indent: 0em, 
    spacing: 1.6em
  )

  set heading(numbering: "1.1")
  show heading.where(level: 1): set block(above: 2em, below: 1.2em)
  show heading.where(level: 2): set block(above: 1.2em, below: 1.2em)
  show heading.where(level: 3): set block(above: 1.2em, below: 1.2em)
  
  show raw.where(block: true): block.with(
    fill: luma(245),
    inset: 10pt,
    radius: 4pt,
    width: 100%,
  )
  show raw: set text(font: "Menlo", size: 0.9em)

  align(center)[
    #text(weight: "bold", size: 1.6em)[#title] \
    #v(0.3em)
    #text(size: 1.2em)[#course] \
    #v(1em)
    #author-name (#author-id) \
    #group #h(1em) #date
  ]
  v(0.3cm)

  body
}

#let sep = box(height: 1.5em)

#let abstract(content) = {
  align(center, text(weight: "bold", size: 1.1em)[Abstract])
  pad(x: 2em, text(size: 0.9em, content))
}

#show: report-template.with(
  title: "Lab 4 Report",
  course: "Robotics Integration Group Project I",
  author-name: "Yuwei ZHAO",
  author-id: "23020036096",
  group: "Group #31",
  date: "2025-11-30",
)

/**************   Abstract   **************/
#abstract[
  Lab4 investigates the mathematical foundations and practical application of polynomial trajectory optimization for UAVs. 
  The primary objective is to formulate trajectory generation as a Quadratic Programming (QP) problem to minimize derivatives of position, such as velocity and snap. 
  We begin by analytically deriving the cost and constraint matrixes for single-segment minimum velocity problems, verifying that the optimal solutions align with the Euler-Lagrange equation. 
  The analysis is then extended to multi-segment minimum snap trajectories, identifying the necessary waypoint, continuity, and boundary constraints required for a unique solution. 
  Finally, these theoretical frameworks are applied to a drone racing scenario, where optimal trajectories are generated to navigate a quadrotor through a sequence of gates.

  See Resources on #link("https://github.com/RamessesN/Robotics_MIT")[github.com/RamessesN/Robotics_MIT].
]

/**************   Introduction   **************/
= Introduction
Trajectory generation is a core component of quadrotor control, ensuring smooth navigation by minimizing specific state derivatives. 
This laboratory focuses on *Polynomial Trajectory Optimization*, specifically transforming the variational problem of minimizing an integral cost into a numerical Quadratic Program (QP).

The report is structured in three parts:
+ *Single-Segment Formulation* \
  We analytically derive the cost matrix $Q$ and constraint matrix $A$ for a minimum velocity problem ($r=1$). We verify that the QP solution aligns with the theoretical optimum derived from the Euler-Lagrange equation.
+ *Multi-Segment Extension* \ 
  We extend the analysis to piece-wise polynomials over $k$ segments. We derive the counting rules for waypoint, continuity, and boundary constraints to ensure a unique solution for high-order problems like Minimum Snap.
+ *Application* \
  Finally, we utilize this framework to generate optimal trajectories for a drone racing scenario, navigating a quadrotor through a sequence of gates.

/**************   Procedure   **************/
= Procedure

== Individual Work

=== Single-segment trajectory optimization

*Consider the following minimum velocity (r = 1) single-segment trajectory optimization problem:
*
*$ min_P(t)  integral_0^1 (P^(\(1\))(t))^2 d t $ <1>
s.t.
$ P(0) = 0, $ <2>
$ P(1) = 1, $ <3>*

*with $P(t) in RR[t]$, i.e., $P(t)$ is a polynomial function in $t$ with real coefficients:
$ P(t) = p_N t^N + p_(N-1) t^(N-1) + ... + p_1 t + p_0 $*

*Note that because of constraint @2 $P(0) = p_0 = 0$, and we can parametrize $P(t)$ without a scalar part $p_0$.*

+ *Suppose we restrict $P(t) = p_1 t$ to be a polynomial of degree 1, what is the optimal solution of problem @1? What is the value of the cost function at the optimal solution?*

  $because P(t) = p_1 t$ \
  Let $t = 1 therefore P(1) = p_1 dot 1 = p_1$ #sep \
  $because P(1) = 1$
  $therefore p_1 = 1$ \
  $therefore "optimal solution:" P(t) = t$. #sep

  $because P(t) = t$
  $therefore P^(\(1\)) t = d / (d t) t = 1$ \
  $therefore "Cost" = integral_0^1 (1)^2 d t = 1$. #sep

+ *Suppose now we allow $P(t)$ to have degree 2, i.e., $P(t) = p_2 t^2 + p_1 t$.*

  - *Write $integral_0^1 (P^(\(1\))(t))^2 d t$, the cost function of problem @1, as $p^T Q p$, where $p = vec(p_1, p_2)$ and $Q in S^2$ is a symmetric $2 crossmark 2$ matrix.*

    $because P(t) = p_2 t^2 + p_1 t$ #sep
    $therefore P^(\(1\))(t) = 2p_2 t + p_1$ \
    $therefore "Cost" = integral_0^1 (2p_2 t + p_1)^2 d t = p_1^2 + 2 p_1 p_2 + 4/3 p_2^2$ #sep \
    In order to write into a $2 crossmark 2$ matrix as $p^T Q p$, we have #sep \
    $vec(p_1 \, p_2) dot mat(Q_11, Q_12; Q_21, Q_22) dot vec(p_1, p_2) = p_1^2 + 2 p_1 p_2 + 4/3 p_2^2$ #sep \
    $therefore Q_(11) = 1, Q_(22) = 4/3 $ #sep \
    $because Q_(12) = Q_(21)$ #sep $therefore 2Q_(12) = 2$ \
    $therefore Q_(12) = Q_(21) = 1 => Q = mat(1, 1; 1, 4/3)$. #sep

  - *Write $P(1) = 1$, constraint @3, as $A p = b$, where $A in RR^(1 crossmark 2)$ and $b in RR$.*

    $because P(1) = 1$ \
    $therefore P(1) = p_2(1)^2 + P_1(1) = p_1 + p_2 = 1$ \
    $because A p = b$ and $p = vec(p_1, p_2)$ #sep
    $therefore A = vec(1 " " 1), b = 1$.

  - *Solve the Quadratic Program (QP):
  $ min_p p^T Q p " " s.t. " " A p = b $ <5>*
  
    *You can solve it by hand, or you can solve it using numerical QP solvers (e.g., you can easily use the `quadprog` function in Matlab). What is the optimal solution you get for $P(t)$, and what is the value of the cost function at the optimal solution? Are you able to get a lower cost by allowing $P(t)$ to have degree 2?*

    $because$ we have $min_p p^T Q p <=> limits(min)_(p_1, p_2)(p_1^2 + 2 p_1 p_2 + 4/3 p_2^2)$ \
    and $A p = b <=> p_1 + p_2 = 1$ #sep \
    Let $p_1 = 1 - p_2$ $therefore "Cost" = (1 - p_2)^2 + 2 (1 - p_2) p_2 + 4/3 p_2^2 = 1 + 1/3 p_2^2$ #sep \
    In order to make `Cost` minimum
    $=> cases(p_1 = 1, p_2 = 0) $ #sep
    $therefore "Cost"_"minimal" = 1$. \
    No, it remains the same value even though `P(t)` has degree 2. #sep

+ *Now suppose we allow $P(t) = p_3 t^3 + p_2 t^2 + p_1 t$:*
  - *Let $p = [p_1. p_2, p_3]^T$, write down $Q in S^3, A in RR^(1 crossmark 3), b in RR$ for QP @5.*

    $because P(t) = p_3 t^3 + p_2 t^2 + p_1 t$ \
    $therefore P_(t)^(\(1\)) = 3 p_3 t^2 + 2 p_2 t + p_1$ #sep \
    $therefore [P_(t)^(\(1\))]^2 = 9 p_3^2 t^4 + 4 p_2^2 t^2 + p_1^2 + underbrace(12 p_2 p_3 t^3, p_2 p_3) + underbrace(6 p_1 p_3 t^1, p_1 p_3) + underbrace(4 p_1 p_2 t, p_1 p_2)$ #sep \
    #set math.equation(numbering: none)
    $therefore$ we have $ cases(
      "item" p_3^2: integral_0^1 9 t^4 d t = 9/5,
      "item" p_2^2: integral_0^1 4 t^2 d t = 4/3,
      "item" p_1^2: integral_0^1 1 d t = 1,
      "item" p_2 p_3: integral_0^1 12 t^3 d t = 3,
      "item" p_1 p_3: integral_0^1 6 t^2 d t = 2,
      "item" p_1 p_2: integral_0^1 4 t d t = 2
    ) $
    $therefore Q = mat(1, 1, 1; 1, 4/3, 3/2; 1, 3/2, 9/5)$ \
    $because p_3(1)^3 + p_2(1)^2 + p_1(1) = 1 => 1 dot p_1 + 1 dot p_2 + 1 dot p_3 = 1$ #sep \
    $therefore A = [1 " " 1 " " 1]," " b = 1$.

  - *Solve the QP, what optimal solution do you get? Do this example agree with the result we learned from Euler-Lagrange equation in class?*

    From the above, we have the path that connects two points and minimizes the change in speed (energy) is always a straight line. That's regardless of the inclusion of higher-degree terms like $t^2 "or" t^3$, the optimization drives their coefficients to 0. The curve connecting the two points that minimizes the velocity cost is always a straight line. Consequently, the value of the cost function remains 1.
    
    #line(length: 100%, stroke: (dash: "dashed"))

    Yes. By `Euler-Lagrange` equation, we have $(partial L) / (partial P) - d / (d t) (partial L) / (partial P') = 0$ \
    Since $L = (P')^2$ #sep $therefore d / (d t) (2P') = 0 => P''(t) = 0$ #sep \ #sep
    The condition $P''(t) = 0$ implies that the optimal function must be linear. The QP result is indeed a linear function, which confirms that the theroretical result derived from calculus of variations.

+ *Now suppose we are interested in adding one more constraint to problem @1:
$ min_P(t)  integral_0^1 (P^(\(1\))(t))^2 d t, " " s.t. P(0) = 0, " " P(1) = 1, " " P^((1))(1) = -2 $ <6>
Using the QP method above, find the optimal solution and optimal cost of problem @6 in the case of:*
  - *$P(t) = p_2 t^2 + p_1 t$, and*
  - *$P(t) = p_3 t^3 + p_2 t^2 + p_1 t$.*

    *#math.section Case I. If $P(t) = p_2 t^2 + p_1 t$,* \ #sep
    #math.cases(
      $"For" P(1) = 1: p_1 + p_2 = 1$,
      $P^(\(1\))(1) = -2: p_1 + 2p_2 = -2$
    ) \
    $therefore$ we have #math.cases(
      $p_1 = 4$,
      $p_2 = -3$
    ) #sep \
    $therefore$ #math.cases(
      $P(t) = -3t^2 + 4t$,
      $"Cost" = p_1^2 + 2 p_1 p_2 + 4/3 p_2^2 = 4$
    ) #sep

    *#math.section Case II. If $P(t) = p_3 t^3 + p_2 t^2 + p_1 t$,* \ #sep
    #math.cases(
      $"For" P(1) = 1: p_1 + p_2 + p_3 = 1$,
      $P^(\(1\))(1) = -2: p_1 + 2p_2 + 3p_3 = -2$
    ) \
    $therefore$ #math.cases(
      $p_1 = 4 + p_3$,
      $p_2 = -3 - 2p_3$
    ) #sep \
    From @3, we have: $Q = mat(1, 1, 1; 1, 4/3, 3/2; 1, 3/2, 9/5)$ #sep \ \
    $p = vec(4 + p_3, -3 - 2p_3, p_3) = underbrace(vec(4, -3, 0), p_"base") + p_3 underbrace(vec(1, -2, 1), d)$ #sep \
    $"For" "Cost"(p_3) = A p_3^2 + B p_3 + C, "we have" cases(A = d^T Q d, B = 2p_"base"^T Q d)$ #sep \
    $therefore Q d = mat(1, 1, 1; 1, 4/3, 3/2; 1, 3/2, 9/5) dot vec(1, -2, 1) = vec(0, -1/6, -1/5)$ #sep \
    $therefore A = 2/15, " "B = 1$ #sep
    $therefore "Cost" = 2/15 p_3^2 + p_3 + 4$ \
    In order to make `Cost` minimum $=> p_3 = -15/4 => "Cost"_"minimal" = 17/8$. #sep
    
=== Multi-segment trajectory optimization

+ *Assume our goal is to compute the minimum snap trajectory ($r = 4$) over $k$ segments. 
  How many and which type of constraints (at the intermediate points and at the start and end of the trajectory) do we need in order to solve this problem? 
  Specify the number of waypoint constraints, free derivative constraints and fixed derivative constraints.*

  #figure(
    image("./source/img/trajOpt.png", width: 60%)
  )

    $"Cost" = integral(x^(\(4\))(t))^2 d t => x^(\(2r\))(t) = 0 => x^(\(8\))(t) = 0$ \ #sep
    *Step 1: Determine the number of Unknowns*

    Given the cost function $"Cost" = integral (x^((4))(t))^2 d t$, the Euler-Lagrange equation yields the necessary condition:
    #set math.equation(numbering: none)
    $ x^((2r))(t) = 0 limits(=>)^(r=4) x^((8))(t) = 0 $
    Integrating this equation *8 times*, we obtain a polynomial of degree $2r-1 = 7$:
    $ P(t) = p_7 t^7 + p_6 t^6 + dots + p_1 t + p_0 $
    $because$ Each segment has $N+1 = 8$ unknown coefficients and there are $k$ segments. \
    $therefore$ Total Unknowns = $8k$, which means we need $8k$ constraints to solve for a unique solution. \
    (1) For _Waypoint Constraints_: #sep \ 
      For each segment $i$, the position at start $t_(i-1)$ and end $t_i$ is fixed. \
      $therefore$ 2 constraints $times$ $k$ segments = $2k$ constraints. \
    (2) For _Free Derivative Constraints_: #sep \
      At the $(k-1)$ intermediate waypoints, the trajectory must be smooth. \
      Continuity is required for derivatives up to $2r-2 = 6$ (i.e., $1^"st"$ to $6^"th"$ derivatives). \
      $therefore$ 6 constraints $times$ $(k-1)$ points = $6(k - 1)$ constraints. \
    (3) For _Fixed Derivative Constraints_: #sep \
      At the start $t_0$ and end $t_k$ of the entire trajectory, we fix derivatives up to $r-1 = 3$ (Velocity, Acc, Jerk). \
      $therefore$ $3$ (start) $+ 3$ (end) = $6$ constraints.

    #line(length: 100%, stroke: (dash: "dashed"))

    *> Proof: * \
    $2k + 6(k - 1) + 6 = 8k$, which confirms that total number of constraints is $8k$.

+ *Can you extend the previous question to the case in which the cost functional minimizes the $r$-th derivative and we have $k$ segments?*

  From the method above, we have $"Total number of constraints" = 2 r k$. \
  Specifically, \
  (1) For _Waypoint Constraints_: #sep \
  $2k$ (Start and End positions for each segment). \
  (2) For _Free Derivative Constraints_: #sep \
   $(k - 1) dot (2r - 2)$ (Continuity of $1^"st" "to" (2r-2)^"th"$ derivatives at intermediate points). \
  (3) For _Fixed Derivative Constraints_: #sep \
  $2(r-1)$ (Fixing $1^"st" "to" (r-1)^"th"$ derivatives at $t_0$ and $t_k$).

  #line(length: 100%, stroke: (dash: "dashed"))

  *> Proof: * \
  $2k + (k - 1)(2r - 2) + 2(r - 1) = 2r k$, which confirms that total number of constraints is $2r k$.

== Team Work

=== Drone Racing

  In this section, we implemented the `trajectory_generation_node` in C++ to enable the quadrotor to autonomously navigate through a sequence of gates. The implementation details are divided into three functional blocks: initialization, trajectory generation, and command publishing.

  + *State Initialization and Safety Hover* \
    The node subscribes to the `/current_state` topic to retrieve the UAV's real-time position and orientation. We converted the ROS `Odometry` messages into `Eigen::Vector3d` for internal calculations. 
    
    Crucially, to prevent the drone from drifting or crashing before the race starts, we implemented a *safety hover logic*. When the trajectory container is empty (idle state), the node continuously captures the current position and publishes it as the desired setpoint with zero velocity and acceleration. This ensures the drone maintains a stable hover at the starting gate until the race track waypoints are received.

  + *Trajectory Optimization and Constraints* \
    Upon receiving the gate waypoints, we utilized the `mav_trajectory_generation` library to formulate the optimization problem.
    - *Position Constraints:* We differentiated between intermediate gates and the endpoints. Intermediate waypoints were added as position constraints allowing non-zero velocity (fly-through), while the final waypoint was constrained using `makeStartOrEnd(...)` to enforce zero velocity and acceleration, ensuring a safe stop.
    - *Yaw Unwrapping Strategy:* A critical challenge was handling the discontinuity of orientation angles (e.g., the jump from $pi$ to $-pi$). We implemented a yaw unwrapping algorithm that checks the difference between consecutive waypoints. If $|psi_k - psi_(k-1)| > pi$, we added or subtracted $2pi$ to the target yaw. This ensures the generated yaw trajectory is continuous and prevents the drone from spinning unnecessary full circles.

  + *Trajectory Sampling and Publishing* \
    The generated polynomial trajectory is continuous, but the controller requires discrete setpoints. We set up a ROS timer running at *100Hz*. At each time step $t$, the node:
    - Checks if $t$ exceeds the total trajectory duration.
    - Evaluates the polynomial to extract the desired position $x_d(t)$, velocity $v_d(t)$, acceleration $a_d(t)$, and yaw $psi_d(t)$.
    - Packages these values into a `MultiDOFJointTrajectoryPoint` message.
    - Converts the `Eigen` types back to `geometry_msgs` and publishes them to the `/desired_state` topic.

=== Simulation Results

  The complete system was tested in the Unity simulator. The drone successfully generated a smooth trajectory passing through all gates and completed the race without collision.

  #figure(
    image("./source/img/testresult.png", width: 80%),
    caption: [The quadrotor autonomously navigating through the gates in the Unity simulator environment.]
  )

  The visualization above confirms that the generated trajectory (visualized by the red path in Rviz/Unity) effectively connects the gate vertices while maintaining smoothness, and the drone's actual path closely tracks this reference.

/**************   Reflection and Analysis   **************/
= Reflection and Analysis

The implementation process highlighted the importance of robust software design in robotics integration. 

1. *Handling Discontinuities:* The mathematical optimization assumes continuous functions. Practical implementation issues, such as the modular nature of angles (Yaw), required specific algorithmic handling (unwrapping) to bridge the gap between linear algebra and physical rotation.

2. *System Timing:* The synchronization between the trajectory generator and the controller was critical. We observed that the sampling frequency of the trajectory node (100Hz) significantly impacted the flight smoothness. A lower frequency caused "stuttering" in the control commands, while a higher frequency ensured the geometric controller received smooth derivatives (velocity and acceleration feed-forward terms), which are essential for aggressive maneuvers.

/**************   Conclusion   **************/
= Conclusion

In this lab, we successfully linked the theoretical framework of polynomial trajectory optimization with a practical ROS-based control system. 

We derived the constraints for single-segment and multi-segment optimization, verifying that Minimum Snap trajectories require $8k$ constraints for a unique solution. In the team work section, we implemented the C++ logic to convert these mathematical constraints into a functional software module. The successful drone racing simulation demonstrated that the QP-based approach effectively generates feasible, smooth, and high-speed trajectories for complex 3D environments.

#pagebreak()

/**************   Source Code   **************/
#set page(header: none, footer: none) 

= Source Code <section:source_code>
- _*trajectory_generation_node.cpp*_
#codly(languages: codly-languages)
```cpp
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/trajectory.h>

#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <eigen3/Eigen/Dense>

class WaypointFollower {
  [[maybe_unused]] ros::Subscriber currentStateSub;
  [[maybe_unused]] ros::Subscriber poseArraySub;
  ros::Publisher desiredStatePub;

  // Current state
  Eigen::Vector3d x;  // current position of the UAV's c.o.m. in the world frame

  ros::Timer desiredStateTimer;

  ros::Time trajectoryStartTime;
  mav_trajectory_generation::Trajectory trajectory;
  mav_trajectory_generation::Trajectory yaw_trajectory;

  void onCurrentState(nav_msgs::Odometry const& cur_state) {    
    // PART 1.1 - 将 ROS 消息转换为 Eigen 向量
    tf::pointMsgToEigen(cur_state.pose.pose.position, x);
  }

  void generateOptimizedTrajectory(geometry_msgs::PoseArray const& poseArray) {
    if (poseArray.poses.size() < 1) {
      ROS_ERROR("Must have at least one pose to generate trajectory!");
      trajectory.clear();
      yaw_trajectory.clear();
      return;
    }

    if (!trajectory.empty()) return;

    const int D = 3;  // dimension of each vertex in the trajectory
    mav_trajectory_generation::Vertex start_position(D), end_position(D);
    mav_trajectory_generation::Vertex::Vector vertices;
    mav_trajectory_generation::Vertex start_yaw(1), end_yaw(1);
    mav_trajectory_generation::Vertex::Vector yaw_vertices;

    // Convert the pose array to a list of vertices
    // Start from the current position and zero orientation
    using namespace mav_trajectory_generation::derivative_order;
    start_position.makeStartOrEnd(x, SNAP);
    vertices.push_back(start_position);
    
    start_yaw.addConstraint(ORIENTATION, 0);
    yaw_vertices.push_back(start_yaw);

    double last_yaw = 0;
    
    for (auto i = 0; i < poseArray.poses.size(); ++i) {
      // PART - 1.2

      // --- 1. Process position vertex ---
      Eigen::Vector3d pos_eigen;
      tf::pointMsgToEigen(poseArray.poses[i].position, pos_eigen);

      mav_trajectory_generation::Vertex pos_vertex(D);
      
      // If it is the last point, it must be the end point, and the velocity acceleration is forced to be 0
      if (i == poseArray.poses.size() - 1) {
          pos_vertex.makeStartOrEnd(pos_eigen, SNAP);
      } else {
          // If it is an intermediate point, only position constraints are added to allow passage at a certain speed
          pos_vertex.addConstraint(POSITION, pos_eigen);
      }
      vertices.push_back(pos_vertex);


      // --- 2. Process yaw vertex ---
      double current_yaw = tf::getYaw(poseArray.poses[i].orientation);

      while (current_yaw - last_yaw > M_PI) current_yaw -= 2 * M_PI;
      while (current_yaw - last_yaw < -M_PI) current_yaw += 2 * M_PI;

      mav_trajectory_generation::Vertex yaw_vertex(1);
      yaw_vertex.addConstraint(ORIENTATION, current_yaw);
      yaw_vertices.push_back(yaw_vertex);

      last_yaw = current_yaw;
    }
    
    std::vector<double> segment_times;
    const double v_max = 15.0;
    const double a_max = 10.0;
    segment_times = estimateSegmentTimes(vertices, v_max, a_max);
    for(int i = 0; i < segment_times.size(); i++) {
      segment_times[i] *= 0.6;
    }

    // Position
    const int N = 10;
    mav_trajectory_generation::PolynomialOptimization<N> opt(D);
    opt.setupFromVertices(vertices, segment_times, mav_trajectory_generation::derivative_order::SNAP);
    opt.solveLinear();

    // Yaw
    mav_trajectory_generation::PolynomialOptimization<N> yaw_opt(1);
    yaw_opt.setupFromVertices(yaw_vertices, segment_times, mav_trajectory_generation::derivative_order::SNAP);
    yaw_opt.solveLinear();

    mav_trajectory_generation::Segment::Vector segments;
    opt.getTrajectory(&trajectory);
    yaw_opt.getTrajectory(&yaw_trajectory);
    trajectoryStartTime = ros::Time::now();

    ROS_INFO("Generated optimizes trajectory from %lu waypoints", vertices.size());
  }

  void publishDesiredState(ros::TimerEvent const& ev) {
    if (trajectory.empty()) {
        // If there is no trajectory yet, publish the current position as the desired position
        trajectory_msgs::MultiDOFJointTrajectoryPoint hover_point;
        
        hover_point.time_from_start = ros::Duration(0.0);

        // 1. Set the position to the current position x
        geometry_msgs::Transform transform;
        tf::vectorEigenToMsg(x, transform.translation);
        
        transform.rotation = tf::createQuaternionMsgFromYaw(0); 
        hover_point.transforms.push_back(transform);

        // 2. Set the speed to 0
        geometry_msgs::Twist velocity;
        velocity.linear.x = 0; velocity.linear.y = 0; velocity.linear.z = 0;
        velocity.angular.x = 0; velocity.angular.y = 0; velocity.angular.z = 0;
        hover_point.velocities.push_back(velocity);

        // 3. Set the acceleration to 0
        geometry_msgs::Twist accel;
        accel.linear.x = 0; accel.linear.y = 0; accel.linear.z = 0;
        accel.angular.x = 0; accel.angular.y = 0; accel.angular.z = 0;
        hover_point.accelerations.push_back(accel);

        // Issue hover commands
        desiredStatePub.publish(hover_point);
        return;
    }

    //  PART 1.3
    trajectory_msgs::MultiDOFJointTrajectoryPoint next_point;
    
    // 1. Calculate the time from the beginning of the trajectory to the present
    ros::Duration time_from_start = ros::Time::now() - trajectoryStartTime;
    next_point.time_from_start = time_from_start; 

    double sampling_time = time_from_start.toSec(); 

    // 2. The time is prevented from exceeding the total length of the trajectory
    if (sampling_time > trajectory.getMaxTime())
      sampling_time = trajectory.getMaxTime();

    // Getting the desired state based on the optimized trajectory we found.
    using namespace mav_trajectory_generation::derivative_order;
    Eigen::Vector3d des_position = trajectory.evaluate(sampling_time, POSITION);
    Eigen::Vector3d des_velocity = trajectory.evaluate(sampling_time, VELOCITY);
    Eigen::Vector3d des_accel = trajectory.evaluate(sampling_time, ACCELERATION);
    Eigen::VectorXd des_orientation = yaw_trajectory.evaluate(sampling_time, ORIENTATION);
    
    // Populate next_point

    // A. Fill Transform (position + pose)
    geometry_msgs::Transform transform;
    tf::vectorEigenToMsg(des_position, transform.translation);
    tf::quaternionTFToMsg(tf::createQuaternionFromYaw(des_orientation(0)), transform.rotation);
    next_point.transforms.push_back(transform);

    // B. Fill Velocity (linear velocity + angular velocity)
    geometry_msgs::Twist velocity;
    tf::vectorEigenToMsg(des_velocity, velocity.linear);
    velocity.angular.x = 0; 
    velocity.angular.y = 0; 
    velocity.angular.z = 0;
    next_point.velocities.push_back(velocity);

    // C. Fill Acceleration (linear acceleration)
    geometry_msgs::Twist accel;
    tf::vectorEigenToMsg(des_accel, accel.linear);
    accel.angular.x = 0;
    accel.angular.y = 0;
    accel.angular.z = 0;
    next_point.accelerations.push_back(accel);
    
    desiredStatePub.publish(next_point);
  }

public:
  explicit WaypointFollower(ros::NodeHandle& nh) {
    currentStateSub = nh.subscribe(
        "/current_state", 1, &WaypointFollower::onCurrentState, this);
    poseArraySub = nh.subscribe("/desired_traj_vertices",
                                1,
                                &WaypointFollower::generateOptimizedTrajectory,
                                this);
    desiredStatePub =
        nh.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>(
            "/desired_state", 1);
    desiredStateTimer = nh.createTimer(
        ros::Rate(100), &WaypointFollower::publishDesiredState, this);
    desiredStateTimer.start();
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "trajectory_generation_node");
  ros::NodeHandle nh;

  WaypointFollower waypointFollower(nh);

  ros::spin();
  return 0;
}
```

- _*controller_node.cpp*_
#codly(languages: codly-languages)
```cpp
#include <ros/ros.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <mav_msgs/Actuators.h>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <cmath>

#define PI M_PI

#include <eigen3/Eigen/Dense>
#include <tf2_eigen/tf2_eigen.h>

class controllerNode{
  ros::NodeHandle nh;

  // PART 1: Declare ROS callback handlers
  ros::Subscriber des_state_sub, cur_state_sub;
  ros::Publisher propeller_speeds_pub;
  ros::Timer control_timer;

  // Controller parameters
  double kx, kv, kr, komega;

  // Physical constants (we will set them below)
  double m;              // mass of the UAV
  double g;              // gravity acceleration
  double d;              // distance from the center of propellers to the c.o.m.
  double cf,             // Propeller lift coefficient
         cd;             // Propeller drag coefficient

  Eigen::Matrix3d J;     // Inertia Matrix
  Eigen::Vector3d e3;    // [0,0,1]
  Eigen::MatrixXd F2W;   // Wrench-rotor speeds map

  // Controller internals
  // Current state
  Eigen::Vector3d x;     // current position of the UAV's c.o.m. in the world frame
  Eigen::Vector3d v;     // current velocity of the UAV's c.o.m. in the world frame
  Eigen::Matrix3d R;     // current orientation of the UAV
  Eigen::Vector3d omega; // current angular velocity of the UAV's c.o.m. in the *body* frame

  // Desired state
  Eigen::Vector3d xd;    // desired position of the UAV's c.o.m. in the world frame
  Eigen::Vector3d vd;    // desired velocity of the UAV's c.o.m. in the world frame
  Eigen::Vector3d ad;    // desired acceleration of the UAV's c.o.m. in the world frame
  double yawd;           // desired yaw angle

  double hz;             // frequency of the main control loop

  static Eigen::Vector3d Vee(const Eigen::Matrix3d& in){
    Eigen::Vector3d out;
    out << in(2,1), in(0,2), in(1,0);
    return out;
  }

  static double signed_sqrt(double val){
    return val > 0 ? sqrt(val) : -sqrt(-val);
  }

public:
  controllerNode():e3(0,0,1),F2W(4,4),hz(1000.0){
      // PART 2: Initialize ROS callback handlers
      xd = Eigen::Vector3d::Zero();
      vd = Eigen::Vector3d::Zero();
      ad = Eigen::Vector3d::Zero();
      yawd = 0.0;
      kx, kv, kr, komega = 0, 0, 0, 0;

      des_state_sub = nh.subscribe("desired_state", 1, &  controllerNode::onDesiredState, this);
      cur_state_sub = nh.subscribe("current_state", 1, &controllerNode::onCurrentState, this);
      propeller_speeds_pub = nh.advertise<mav_msgs::Actuators>("/rotor_speed_cmds", 1);
      control_timer = nh.createTimer(ros::Duration(1.0/hz), &controllerNode::controlLoop, this);

      // PART 6: Tune your gains!
      nh.getParam("kx", kx);
      nh.getParam("kv", kv);
      nh.getParam("kr", kr);
      nh.getParam("komega", komega);
      ROS_INFO("Gain values:\nkx: %f \nkv: %f \nkr: %f \nkomega: %f\n", kx, kv, kr, komega);

      // Initialize constants
      m = 1.0;
      cd = 1e-5;
      cf = 1e-3;
      g = 9.81;
      d = 0.3;
      J << 1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0;

      // F2W matrix
      double d_by_sqrt2 = d/std::sqrt(2.0);
      F2W <<
          cf,            cf,            cf,            cf,
          cf*d_by_sqrt2, cf*d_by_sqrt2,-cf*d_by_sqrt2,-cf*d_by_sqrt2,
         -cf*d_by_sqrt2, cf*d_by_sqrt2, cf*d_by_sqrt2,-cf*d_by_sqrt2,
          cd,           -cd,            cd,           -cd;
  }

  void onDesiredState(const trajectory_msgs::MultiDOFJointTrajectoryPoint& des_state){
      //  PART 3: Objective - fill in xd, vd, ad, yawd
      xd << des_state.transforms[0].translation.x, 
            des_state.transforms[0].translation.y, 
            des_state.transforms[0].translation.z;
            
      vd << des_state.velocities[0].linear.x, 
            des_state.velocities[0].linear.y, 
            des_state.velocities[0].linear.z;
            
      ad << des_state.accelerations[0].linear.x, 
            des_state.accelerations[0].linear.y, 
            des_state.accelerations[0].linear.z;

      tf2::Quaternion quat;
      tf2::fromMsg(des_state.transforms[0].rotation, quat);
      yawd = tf2::getYaw(quat);
  }

  void onCurrentState(const nav_msgs::Odometry& cur_state){
      // PART 4: Objective - fill in x, v, R and omega
      // Position
      x << cur_state.pose.pose.position.x, 
          cur_state.pose.pose.position.y, 
          cur_state.pose.pose.position.z;

      // Velocity
      v << cur_state.twist.twist.linear.x, 
          cur_state.twist.twist.linear.y, 
          cur_state.twist.twist.linear.z;

      // Orientation
      tf2::Quaternion quat;
      tf2::fromMsg(cur_state.pose.pose.orientation, quat);
      Eigen::Quaterniond eigen_quat(quat.w(), quat.x(), quat.y(), quat.z());
      eigen_quat.normalize();
      R = eigen_quat.toRotationMatrix();

      // Angular velocity
      Eigen::Vector3d omega_world;
      omega_world << cur_state.twist.twist.angular.x, 
                    cur_state.twist.twist.angular.y, 
                    cur_state.twist.twist.angular.z;

      omega = R.transpose() * omega_world;
  }

  void controlLoop(const ros::TimerEvent& t){
    Eigen::Vector3d ex, ev, er, eomega;
    // PART 5: Objective - Implement the controller!
    ex = x - xd; // position error
    ev = v - vd; // velocity error

    // Rd matrix
    Eigen::Vector3d F_des = -kx*ex - kv*ev + m*g*e3 + m*ad;
    Eigen::Vector3d b3d = F_des.normalized();
    Eigen::Vector3d b1d_desired(cos(yawd), sin(yawd), 0);

    Eigen::Vector3d b2d = (b3d.cross(b1d_desired)).normalized();
    Eigen::Vector3d b1d = (b2d.cross(b3d)).normalized();

    Eigen::Matrix3d Rd;
    Rd.col(0) = b1d;
    Rd.col(1) = b2d;
    Rd.col(2) = b3d;
    
    er = 0.5 * Vee(Rd.transpose() * R - R.transpose() * Rd); // Orientation error
    eomega = omega; // Rotation-rate error
    
    // Desired wrench
    double f = (-kx * ex + -kv * ev + m * g * e3 + m * ad).dot(R * e3);
    Eigen::Vector3d M = -kr * er - komega * eomega + omega.cross(J * omega);

    // Recover the rotor speeds from the wrench
    Eigen::Vector4d W;
    W << f, M.x(), M.y(), M.z();
    Eigen::Vector4d omega_sq = F2W.colPivHouseholderQr().solve(W);

    Eigen::Vector4d rotor_speeds;
    for (int i = 0; i < 4; i++) {
        rotor_speeds(i) = signed_sqrt(omega_sq[i]);
    }

    // Populate and publish the control message
    mav_msgs::Actuators control_msg;
    control_msg.angular_velocities.clear();
    for (int i = 0; i < 4; i++) {
        control_msg.angular_velocities.push_back(rotor_speeds(i));
    }
    propeller_speeds_pub.publish(control_msg);
  }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "controller_node");
  controllerNode n;
  ros::spin();
}
```