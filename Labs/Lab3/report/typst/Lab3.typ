/**************   Report Template   **************/
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
  See Resources on #link("https://github.com/RamessesN/Robotics_MIT")[github.com/RamessesN/Robotics_MIT].
]

/**************   Introduction   **************/
= Introduction
// TODO

/**************   Procedure   **************/
= Procedure

== Individual Work

=== Transformations in Practice

+ *MESSAGE VS. TF*
  - *Assume we have an incoming `geometry_msgs::Quaternion quat_msg` that holds the pose of our robot. We need to save it in an already defined `tf2::Quaternion quat_tf` for further calculations. Write one line of C++ code to accomplish this task.* \
    ```cpp
    tf2::fromMsg(quat_msg, quat_tf);
    ```

    More specifically, we can find the official documentation of `fromMsg()` at #link("https://docs.ros.org/en/noetic/api/tf2_geometry_msgs/html/c++/namespacetf2.html#a2fdf91676912e510c0002aa66dde2800")[this page]:

    #figure(
      image("../source/img/deliverable_1/from_msg.png", width: 100%),
      caption: [tf2 Quaternion doc]
    ) <fig:tf2_fromMsg_doc>

  - *Assume we have just estimated our robot’s newest rotation and it’s saved in a variable called `quat_tf` of type `tf2::Quaternion`. Write one line of C++ code to convert it to a `geometry_msgs::Quaternion` type. Use `quat_msg` as the name of the new variable.* \
    ```cpp
    geometry_msgs::Quaternion quat_msg = tf2::toMsg(quat_tf);
    ```

    More specifically, we can find the official documentation of `toMsg()` in the same #link("https://docs.ros.org/en/noetic/api/tf2_geometry_msgs/html/c++/namespacetf2.html#a2fdf91676912e510c0002aa66dde2800")[link] as `fromMsg()`:

    #figure(
      image("../source/img/deliverable_1/to_msg.png", width: 100%),
      caption: [geometry_msgs Quaternion doc]
    ) <fig:tf2_toMsg_doc>

  - *If you just want to know the scalar value of a `tf2::Quaternion`, what member function will you use?*
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
    ```cpp
    double yaw = tf2::getYaw(quat_t);
    ```

    More specifically, the doc of `getYaw()` is shown at #link("https://docs.ros.org/en/noetic/api/tf2/html/namespacetf2.html")[this page]:

    #figure(
      image("../source/img/deliverable_1/get_yaw.png", width: 100%),
      caption: [Quaternion get_yaw doc]
    ) <fig:quaternion_getYaw_doc>

  - *Assume you have a `geometry_msgs::Quaternion quat_msg`. How to you convert it to an Eigen 3-by-3 matrix? Refer to #link("https://docs.ros.org/en/jade/api/tf2_eigen/html/index.html")[this] for possible functions. You probably need two function calls for this.*
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
+ *STRUCTURE OF QUADROTORS* \
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
=== Launching the TESSE simulator with ROS bridge
=== Implement the controller
=== Simulator conventions
=== Geometric controller for the UAV

/**************   Reflection and Analysis   **************/
= Reflection and Analysis

/**************   Conclusion   **************/
= Conclusion

#pagebreak()

/**************   Source Code   **************/
#set page(header: none, footer: none) 

= Source Code <section:source_code>
- 
-