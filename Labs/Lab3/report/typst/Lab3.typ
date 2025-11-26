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
        set text(size: 10pt, font: "Times New Roman")
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
  show raw: set text(font: "Courier New", size: 0.9em)

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

#let abstract(content) = {
  align(center, text(weight: "bold", size: 1.1em)[Abstract])
  pad(x: 2em, text(size: 0.9em, content))
  v(1em)
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
      image("../source/img/from_msg.png", width: 100%),
      caption: [tf2 Quaternion doc]
    ) <fig:tf2_fromMsg_doc>

  - *Assume we have just estimated our robot’s newest rotation and it’s saved in a variable called `quat_tf` of type `tf2::Quaternion`. Write one line of C++ code to convert it to a `geometry_msgs::Quaternion` type. Use `quat_msg` as the name of the new variable.* \
    ```cpp
    geometry_msgs::Quaternion quat_msg = tf2::toMsg(quat_tf);
    ```

    More specifically, we can find the official documentation of `toMsg()` in the same #link("https://docs.ros.org/en/noetic/api/tf2_geometry_msgs/html/c++/namespacetf2.html#a2fdf91676912e510c0002aa66dde2800")[link] as `fromMsg()`:

    #figure(
      image("../source/img/to_msg.png", width: 100%),
      caption: [geometry_msgs Quaternion doc]
    ) <fig:tf2_toMsg_doc>

  - *If you just want to know the scalar value of a `tf2::Quaternion`, what member function will you use?*
  ```cpp
  double scalar = quat_tf.getW();
  ```

  More specifically, we find the official documentation of `getW()` #link("https://docs.ros.org/en/noetic/api/tf2/html/classtf2_1_1Quaternion.html#af21a6d7d4d7f29476307456a0cb020cb")[here]:

  #figure(
    image("../source/img/get_w.png", width: 100%),
    caption: [Quaternion get_w doc]
  ) <fig:quaternion_getW_doc>

+ *CONVERSION*
  - *Assume you have a `tf2::Quaternion quat_t`. How to extract the yaw component of the rotation with just one function call?*
    ```cpp
    double yaw = tf2::getYaw(quat_t);
    ```

    More specifically, the doc of `getYaw()` is shown at #link("https://docs.ros.org/en/noetic/api/tf2/html/namespacetf2.html")[this page]:

    #figure(
      image("../source/img/get_yaw.png", width: 100%),
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
      image("../source/img/to_rotation_matrix.png", width: 100%),
      caption: [Eigen toRotationMatrix doc]
    ) <fig:eigen_to_rotation_matrix_doc>

=== Modelling and control of UAVs
+ *STRUCTURE OF QUADROTORS*
+ *CONTROL OF QUADROTORS*

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