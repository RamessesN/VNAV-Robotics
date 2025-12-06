/**************   Report Template   **************/
#import "@preview/codly:1.3.0": *
#import "@preview/codly-languages:0.1.1": *

#show: codly-init.with()

#let table-continued = state("table-continued", false)

#let continued-header(cols) = {
  table.cell(colspan: cols, stroke: none, inset: 0pt, {
    context if table-continued.get() {
      align(right, text(size: 0.8em, style: "italic", fill: luma(100))[Table (continued)])
      v(5pt)
    } else {
      table-continued.update(true)
      v(-1em)
    }
  })
}

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
  title: "Lab 3 Report",
  course: "Robotics Integration Group Project I",
  author-name: "Yuwei ZHAO",
  author-id: "23020036096",
  group: "Group #31",
  date: "2025-11-26",
)

/**************   Abstract   **************/
#abstract[
  Lab3 explores the fundamentals of rigid body transformations and the implementation of a geometric controller for a quadrotor UAV. It separated into the following two parts: *(1)* The individual part focuses on the practical manipulation of rotations using `tf2`, `geometry_msgs`, and `Eigen` libraries within the ROS ecosystem, alongside a theoretical analysis of quadrotor dynamics and drag compensation. *(2)* The collaborative component involves implementing a non-linear geometric controller on SE(3) to track dynamic trajectories. The system is integrated and validated using the TESSE Unity simulator, demonstrating the UAV's ability to follow a reference path effectively.

  See Resources on #link("https://github.com/RamessesN/Robotics_MIT")[github.com/RamessesN/Robotics_MIT].
]

/**************   Introduction   **************/
= Introduction
Precise trajectory tracking is a fundamental challenge in UAV robotics due to the system's fast and underactuated dynamics. This laboratory aims to bridge the gap between theoretical control derivation and practical software implementation within the ROS1.

The objectives of this report are threefold: *(1)* First, we practice handling rigid body transformations using `tf2` and `Eigen` libraries to ensure accurate state estimation. *(2)* Second, we analyze quadrotor dynamics, specifically examining the mixing matrix and drag force compensation. *(3)* Finally, the report details the implementation of a geometric tracking controller on the Special Euclidean group $"SE"(3)$, based on the work of Lee et al. We integrate this controller into the MIT-TESSE simulation environment to validate its performance. The procedure demonstrates the complete workflow from mathematical modelling to C++ implementation, culminating in the successful tracking of a complex trajectory.

/**************   Procedure   **************/
= Procedure

== Individual Work

=== Transformations in Practice
  + *MESSAGE VS. TF*
    - *Assume we have an incoming `geometry_msgs::Quaternion quat_msg` that holds the pose of our robot. We need to save it in an already defined `tf2::Quaternion quat_tf` for further calculations. Write one line of C++ code to accomplish this task.* \
      #codly(languages: codly-languages)
      ```cpp
      tf2::fromMsg(quat_msg, quat_tf);
      ```

      More specifically, we can find the official documentation of `fromMsg()` at #link("https://docs.ros.org/en/noetic/api/tf2_geometry_msgs/html/c++/namespacetf2.html#a2fdf91676912e510c0002aa66dde2800")[this page]:

      #figure(
        image("../source/img/deliverable_1/from_msg.png", width: 100%),
        caption: [tf2 Quaternion doc]
      ) <fig:tf2_fromMsg_doc>

    - *Assume we have just estimated our robot’s newest rotation and it’s saved in a variable called `quat_tf` of type `tf2::Quaternion`. Write one line of C++ code to convert it to a `geometry_msgs::Quaternion` type. Use `quat_msg` as the name of the new variable.* \
      #codly(languages: codly-languages)
      ```cpp
      geometry_msgs::Quaternion quat_msg = tf2::toMsg(quat_tf);
      ```

      More specifically, we can find the official documentation of `toMsg()` in the same #link("https://docs.ros.org/en/noetic/api/tf2_geometry_msgs/html/c++/namespacetf2.html#a2fdf91676912e510c0002aa66dde2800")[link] as `fromMsg()`:

      #figure(
        image("../source/img/deliverable_1/to_msg.png", width: 100%),
        caption: [geometry_msgs Quaternion doc]
      ) <fig:tf2_toMsg_doc>

    - *If you just want to know the scalar value of a `tf2::Quaternion`, what member function will you use?*
    #codly(languages: codly-languages)
    ```cpp
    double scalar = quat_tf.getW();
    ```

    More specifically, we find the official documentation of `getW()` #link("https://docs.ros.org/en/noetic/api/tf2/html/classtf2_1_1Quaternion.html#af21a6d7d4d7f29476307456a0cb020cb")[here]:

    #figure(
      image("../source/img/deliverable_1/get_w.png", width: 100%),
      caption: [Quaternion get_w doc]
    ) <fig:quaternion_getW_doc>

  + *CONVERSION*
    - *Assume you have a `tf2::Quaternion quat_t`. How to extract the yaw component of the rotation with just one function call?*
      #codly(languages: codly-languages)
      ```cpp
      double yaw = tf2::getYaw(quat_t);
      ```

      More specifically, the doc of `getYaw()` is shown at #link("https://docs.ros.org/en/noetic/api/tf2/html/namespacetf2.html")[this page]:

      #figure(
        image("../source/img/deliverable_1/get_yaw.png", width: 100%),
        caption: [Quaternion get_yaw doc]
      ) <fig:quaternion_getYaw_doc>

    - *Assume you have a `geometry_msgs::Quaternion quat_msg`. How to you convert it to an Eigen 3-by-3 matrix? Refer to #link("https://docs.ros.org/en/jade/api/tf2_eigen/html/index.html")[this] for possible functions. You probably need two function calls for this.*
      #codly(languages: codly-languages)
      ```cpp
      #include <tf2_eigen/tf2_eigen.h>

      Eigen::Quaterniond eigen_quat;

      // The first function to call
      tf2::fromMsg(quat_msg, eigen_quat);
      // The second function to call
      Eigen::Matrix3d eigen_mat3 = eigen_quat.toRotationMatrix();
      ```

      More specifically, the doc of `toRotationMatrix()` can be found #link("https://libeigen.gitlab.io/eigen/docs-5.0/classEigen_1_1QuaternionBase.html#a8cf07ab9875baba2eecdd62ff93bfc3f")[here]:

      #figure(
        image("../source/img/deliverable_1/to_rotation_matrix.png", width: 100%),
        caption: [Eigen toRotationMatrix doc]
      ) <fig:eigen_to_rotation_matrix_doc>

=== Modelling and control of UAVs
  + *STRUCTURE OF QUADROTORS*
    #figure(
      image("../source/img/deliverable_2/drone_spinning.png", width: 50%),
    ) <fig:drone_spinning>

    *The figure above depicts two quadrotors (a) and (b). Quadrotor (a) is a fully functional UAV, while for Quadrotor (b) someone changed propellers 3 and 4 and reversed their respective rotation directions.*

    *Show mathematically that quadrotor (b) is not able to track a trajectory defined in position $[x, y, z]$ and yaw orientation $Psi$.*

    In order to proof quadroter (b) is not able to track a trajectory, we have to judge whether the rank of the matrix #emph[*F*] is full.

    The quadrotor has four inputs - the thrust of 4 motors: $f_1$, $f_2$, $f_3$, $f_4$, and four outputs free degrees (total thrust #emph[T] and three-axis torque $tau#sub[roll]$, $tau#sub[pitch]$, and $tau#sub[yaw]$). The linear equation is: 
    $ u = #emph[F]f $
    which $u = vec(T \, tau#sub[roll] \, tau#sub[pitch] \, tau#sub[yaw])^T$ is the output vector, $f=vec(f_1 \, f_2 \, f_3 \, f_4)^T$ is the input vector.

    #figure(
      table(
        columns: (auto, auto, auto),
        inset: 10pt,
        align: horizon,
        table.header(
          [], [*Quadrotor (a)*], [*Quadrotor (b)*]
        ),
        [*Thrust $T$*], $vec(1 \, 1 \, 1 \, 1)$, $vec(1 \, 1 \, 1 \, 1)$,
        [*Roll $tau_"roll"$*], $vec(-d \, -d \, d \, d)$, $vec(-d \, -d \, d \, d)$,
        [*Pitch $tau_"pitch"$*], $vec(d \, -d \, -d \, d)$, $vec(d \, -d \, -d \, d)$,
        [*Yaw $tau_"yaw"$*], $vec(-c \, c \, -c \, c)$, $vec(-c \, c \, c \, -c)$,
        [*Matrix #emph[F]*], [$F_a = mat(1, 1, 1, 1; -d, -d, d, d; d, -d, -d, d; -c, c, -c, c)$], [$F_b = mat(1, 1, 1, 1; -d, -d, d, d; d, -d, -d, d; -c, c, c, -c)$],
        [*Rank*], [$4$ (full)], [$3$ (not full)]
      ),
      caption: [Comparison of Quadrotors],
    ) <tab:quad_comparison>
    
    $because "rank"_b = 3 < 4 therefore "the matrix" F_b "isn't full rank"$, which means that the output space of the system has only 3 dimensions so that it is not able to track a trajectory defined with $[x, y, z, Psi]$.

  + *CONTROL OF QUADROTORS* \
    *Assume that empirical data suggest you can approximate the drag force (in the body frame) of a quadrotor body as:* \
    *$ F^b = mat(
      0.1, 0, 0;
      0, 0.1, 0;
      0, 0, 0.2
    ) (v^b)^2 $*
    *With $(v^b)^2 = vec(
      -v_x^b dot |v_x^b| \, -v_y^b dot |v_y^b| \, -v_z^b dot |v_z^b|
    )^T ,$ and $v_x, v_y, v_z$ being the quadrotor velocities along the axes of the body frame.*

    *With the controller discussed in class (see referenced paper #super[1]), describe how you could use the information above to improve the tracking performance.*

    From the #link("https://mathweb.ucsd.edu/~mleok/pdf/LeLeMc2010_quadrotor.pdf")[referenced paper], we have:

    $ cases(
    m dot(v) &= m g e_3 - f R e_3, #sep
    
    arrow(b)_(3d) &= - (-k_x e_x - k_v e_v - m g e_3 + m diaer(x)_d) / (|| -k_x e_x - k_v e_v - m g e_3 + m diaer(x)_d ||), #sep
    
    f &= -(-k_x e_x - k_v e_v - m g e_3 + m diaer(x)_d) dot.c R e_3
    ) $

    $therefore$ Expected resultant force vector: $u_("nominal") = -k_x e_x - k_v e_v - m g e_3 + m diaer(x)_d$

    $because$ Drag Force: $D_("inertial") = R dot F_("drag")^b = R dot (mat(0.1, 0, 0; 0, 0.1, 0; 0, 0, 0.2) (v_b)_("signed")^2)$

    $therefore$ $u_("new") = u_("nominal") - D_("inerital") = -k_x e_x - k_v e_v - m g e_3 + m diaer(x)_d - R dot F_("drag")^b$

    $therefore$ we have 
    $cases(
      arrow(b)_(3d) = - (u_("new")) / (||u_("new")||), #sep

      f = -u_("new") dot R e^3
    )$ .

== Team Work
=== Trajectory tracking for UAVs
  This section is to set up the environment, launch the simulator with ROS bridge, and implement the geometric controller for trajectory tracking.
  
  First, we need to set up the workspace and install the dependencies.
  #codly(languages: codly-languages)
  ```sh
  sudo apt install ros-noetic-ackermann-msgs
  ```
  Then, change to the `labs` directory, pull the latest code, and copy the lab3 files to the ROS workspace.
  #codly(languages: codly-languages)
  ```sh
  cd ~/labs
  git pull

  cp -r ~/labs/lab3/. ~/vnav_ws/src
  cd ~/vnav_ws
  ```
  Install the `tesse-interface` package and clone the `mav_comm` repository.
  #codly(languages: codly-languages)
  ```sh
  cd ~/vnav_ws/src/tesse-ros-bridge/tesse-interface
  pip install -r requirements.txt # Dependencies
  pip install .

  cd ~/vnav_ws/src && git clone https://github.com/ethz-asl/mav_comm.git
  ```
  Build the workspace and make the simulator executable.
  #codly(languages: codly-languages)
  ```sh
  catkin build
  source devel/setup.bash

  cd ~/vnav-builds/lab3/
  chmod +x lab3.x86_64
  ```

=== Launching the TESSE simulator with ROS bridge
  This section is to launch the TESSE simulator with ROS bridge and visualize the UAV in RViz.
  First, we need to run the simulator and the ROS bridge.
  #codly(languages: codly-languages)
  ```sh
  cd ~/vnav-builds/lab3/
  ./lab3.x86_64

  source devel/setup.zsh
  roslaunch tesse_ros_bridge tesse_quadrotor_bridge.launch
  ```
  Then, we can open RViz with the provided configuration file to visualize the UAV and its trajectory.
  #codly(languages: codly-languages)
  ```sh
  cd ~/vnav_ws/src/controller_pkg
  rviz -d rviz/lab3.rviz
  ```

=== Implement the controller
  This section describes the implementation of the geometric controller for the UAV in C++. The controller subscribes to the desired and current state topics and publishes the rotor speed commands.

  #let table-continued = state("table-continued", false)

  #show table: it => table-continued.update(false) + it

  #table(
    columns: (auto, auto, 1fr),
    inset: 8pt,
    stroke: 0.5pt + luma(200),
    align: (x, y) => if x < 2 { center + horizon } else { left + horizon },
    
    table.header(
      table.cell(colspan: 3, stroke: none, inset: 0pt, {
        context if table-continued.get() {
          align(left, text(size: 0.8em, style: "italic", fill: luma(100))[Table (continued)])
          v(5pt)
        } else {
          table-continued.update(true)
          v(-1em)
        }
      }),

      table.cell(fill: luma(230))[*Topic Name*], 
      table.cell(fill: luma(230))[*Message Type*], 
      table.cell(fill: luma(230))[*Structure & Description*],
    ),

    raw("/desired_state"), 
    raw("trajectory_msgs/\nMultiDOFJoint\nTrajectoryPoint"),
    [
      *`transforms`* (`geometry_msgs/Transform[]`):
        - `translation`: 3D vector, desired position (World frame).
        - `rotation`: Quaternion, desired orientation (Yaw only).
      *`velocities`* (`geometry_msgs/Twist[]`):
        - `linear`: 3D vector, desired velocity (World frame).
        - `angular`: _Ignored_.
      *`accelerations`* (`geometry_msgs/Twist[]`):
        - `linear`: 3D vector, desired acceleration (World frame).
        - `angular`: _Ignored_.
    ],

    raw("/current_state"), 
    raw("nav_msgs/Odometry"),
    [
      *`pose.pose`* (`geometry_msgs/Pose`):
        - `position`: 3D vector, current position (World frame).
        - `orientation`: Quaternion, current UAV orientation.
      *`twist.twist`* (`geometry_msgs/Twist`):
        - `linear`: 3D vector, current linear velocity (World frame).
        - `angular`: 3D vector, current angular velocity (*World frame*).
    ],

    raw("/rotor_speed_cmds"), 
    raw("mav_msgs/Actuators"),
    [
      *`angular_velocities`* (`float64[]`):
        - Array containing the desired speeds of the propellers.
    ]
  )

=== Simulator conventions
  There are three key differences between the mathematical model presented in the reference paper and the conventions used in the TESSE simulator (and standard ROS frames). 
  We have adapted the controller implementation to account for these differences:
  + *Reference Frame (Z-axis direction)*:
    - The paper assumes a Z-down coordinate system where the gravity vector is positive along the $z$-axis ($m g e_3$).
    - The simulator uses the standard ROS ENU (East-North-Up) frame where the $z$-axis points upward. Consequently, the gravity compensation term in the desired force calculation must be inverted.
    - *Modification*: In Equation (12) of the paper, the term $-m g e_3$ is replaced by $+m g e_3$ to counteract gravity in the Z-up frame.

  + *Motor Configuration*:
    - The paper assumes a `+` configuration where the body X-axis is aligned with one of the rotors. This results in a mixing matrix where pitch and roll moments are decoupled.
    - The simulator uses an `x` configuration where the body axes are offset by $45^circle.stroked.tiny$ from the rotor arms. This means all four rotors contribute to both roll and pitch moments.
    - *Modification*: The mixing matrix (mapping from rotor forces to body wrench) is adapted. Let $L = d / sqrt(2)$ be the projected arm length, the mapping becomes:
    #{
      set math.equation(numbering: none)
      $ mat(f; M_x; M_y; M_z) = mat(
        1, 1, 1, 1;
        -L, L, L, -L;
        -L, -L, L, L;
        -kappa, kappa, -kappa, kappa
      ) dot vec(f_1, f_2, f_3, f_4) $
    }

  + *Aerodynamic Coefficients*:
    - The paper defines a single ratio coefficient $c_(tau f)$ relating torque to thrust.
    - The simulator specifies separate lift coefficient $c_f$ and drag coefficient $c_d$.
    - *Modification*: We define $kappa = c_d / c_f$ to match the term $c_(tau f)$ used in the paper's derivation.

=== Geometric controller for the UAV
  #figure(
    image("../source/img/teamwork/live_demo.png", width: 80%),
    caption: [UAV trajectory tracking in TESSE simulator]
  ) <fig:uav_trajectory_tracking>
  The figure above demonstrated the successful implementation of the geometric controller for trajectory tracking in the TESSE simulator and the detailed implementation is described below.

  Geometric controller part is encapsulated within the `controlLoop` function of the `controllerNode` class. The control logic follows the tracking control on the Special Euclidean Group $"SE"(3)$ proposed by Lee et al., adapted for the ENU (East-North-Up) reference frame used in the TESSE simulator.

  The control process is divided into four main steps:

  + *Translational Dynamics Control* \
    First, we compute the position and velocity errors:
    #{
      set math.equation(numbering: none)
      $ e_x = x - x_d, quad e_v = v - v_d $
    }
    
    The desired force vector $F_"des"$ is computed to stabilize the translational error. Note that unlike the reference paper which assumes a Z-down frame, our simulation uses a Z-up frame. Therefore, the gravity compensation term is positive ($+m g e_3$) to provide an upward force:
    #{
      set math.equation(numbering: none)
      $ F_"des" = -k_x e_x - k_v e_v + m g e_3 + m a_d $
    }
    
    This vector defines the desired direction of the body $z$-axis ($b_(3d)$), which represents the thrust direction:
    #{
      set math.equation(numbering: none)
      $ b_(3d) = F_"des" / (||F_"des"||) $
    }

  + *Attitude Generation* \
    We construct the desired rotation matrix $R_d = [b_(1d), b_(2d), b_(3d)]$ to align the thrust vector while maintaining the desired yaw angle $psi_d$.
    
    Let $b_(1d)^"des" = [cos psi_d, sin psi_d, 0]^T$ be the desired heading. The orthogonal body axes are computed via cross products:
    #{
      set math.equation(numbering: none)
      $ b_(2d) = (b_(3d) times b_(1d)^"des") / (||b_(3d) times b_(1d)^"des"||), quad b_(1d) = b_(2d) times b_(3d) $
    }
    
    This ensures $R_d in "SO"(3)$ is orthonormal and respects the desired yaw constraint projected onto the plane perpendicular to the thrust.

  + *Rotational Dynamics Control* \
    We calculate the attitude error $e_R$ and angular velocity error $e_Omega$ in the body frame. The function `Vee()` implements the map from $s o(3)$ to $RR^3$:
    #{
      set math.equation(numbering: none)
      $ e_R = 1/2 (R_d^T R - R^T R_d)^or $
      $ e_Omega = Omega - R^T R_d Omega_d approx Omega quad ("Assuming" Omega_d approx 0 "for stabilization") $
    }
    
    The control outputs are the scalar total thrust $f$ and the moment vector $M$:
    #{
      set math.equation(numbering: none)
      $ f = F_"des" dot (R e_3) $
      $ M = -k_R e_R - k_Omega e_Omega + Omega times (J Omega) $
    }

  + *Control Allocation* \
    Finally, we map the computed wrench $(f, M_x, M_y, M_z)$ to individual rotor speeds. The simulator simulates a quadrotor in an `X` configuration. The relationship between rotor speeds squared ($omega_i^2$) and the wrench is modeled by the mixing matrix $cal(A)$:
    #{
      set math.equation(numbering: none)
      $ vec(f, M_x, M_y, M_z) = cal(A) dot vec(omega_1^2, omega_2^2, omega_3^2, omega_4^2) $
    }
    In our implementation (`F2W` matrix), considering the arm length projection $L = d/sqrt(2)$ and the aerodynamic coefficients $c_f, c_d$:
    #{
      set math.equation(numbering: none)
      $ cal(A) = mat(
        c_f, c_f, c_f, c_f;
        L c_f, L c_f, -L c_f, -L c_f;
        -L c_f, L c_f, L c_f, -L c_f;
        c_d, -c_d, c_d, -c_d
      ) $
    }
    
    We solve this linear system using `F2W.colPivHouseholderQr().solve(W)` to obtain the squared angular velocities, then compute the square root (preserving signs) to get the final commands sent to the `/rotor_speed_cmds` topic.

/**************   Reflection and Analysis   **************/
= Reflection and Analysis
  In this lab, we bridged the gap between theoretical nonlinear control and practical implementation on a robotic system. Several key insights and challenges emerged during the process:

  + *The Importance of Coordinate Consistency* \
    One of the most critical challenges was reconciling the mathematical conventions in Lee et al.'s paper (NED frame, `+` configuration) with the simulation environment (ENU frame, `X` configuration).
    - We observed that a direct transcription of the paper's formulas led to immediate instability.
    - Specifically, the gravity compensation term required a sign inversion ($+m g e_3$) to account for the Z-up frame.
    - Furthermore, the mixing matrix `F2W` had to be re-derived for the `X` configuration. Incorrect signs in the mixing matrix caused the drone to flip immediately upon takeoff, highlighting that geometric correctness is useless without correct physical mapping.

  + *Numerical Stability of Rotations* \
    We encountered significant high-frequency vibrations during initial testing. Through analysis, we identified the cause as numerical errors in the quaternion representation.
    - The `Eigen::Quaternion` constructor does not automatically normalize the input. When converting `nav_msgs::Odometry` (which contains slight sensor/integration noise) to a rotation matrix without normalization, the resulting matrix $R$ was no longer strictly orthogonal ($R in.not "SO"(3)$).
    - This violated the geometric controller's assumption, causing the error term $e_R$ to generate erroneous feedback. Adding an explicit `eigen_quat.normalize()` step completely eliminated the vibrations, proving that strict adherence to mathematical constraints is vital in geometric control.

  + *Cascaded Control Dynamics* \
    The tuning process revealed the distinct time-scale separation required for the cascaded architecture.
    - The inner loop (attitude control, gains $k_R, k_Omega$) must be significantly faster than the outer loop (position control, gains $k_x, k_v$).
    - If $k_x$ was too high relative to $k_R$, the drone would request aggressive attitude changes that it could not track, leading to overshoot. We found that a ratio where attitude dynamics are roughly 3-5 times faster than position dynamics yielded the most stable tracking performance.

/**************   Conclusion   **************/
= Conclusion
This lab successfully demonstrated the end-to-end workflow of an autonomous aerial robotics system, ranging from mathematical modeling to `C++` software implementation.

We first derived the solutions for Polynomial Trajectory Optimization, verifying that high-order polynomials are necessary to satisfy boundary constraints and ensure smoothness for aggressive maneuvers. We then implemented a Geometric Tracking Controller on $"SE"(3)$, which avoids the singularities associated with Euler angles and provides almost global asymptotic stability.

By integrating these components into the ROS / TESSE ecosystem, we successfully navigated a quadrotor through a complex race course. The experiments highlighted that while mathematical derivation provides the foundation, robust robot autonomy relies equally on handling implementation details—such as coordinate transformations, numerical conditioning, and proper gain tuning. The resulting system is capable of precise dynamic tracking, validating the effectiveness of geometric control for underactuated mechanical systems.

#pagebreak()

/**************   Source Code   **************/
#set page(header: none, footer: none) 

= Source Code <section:source_code>
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