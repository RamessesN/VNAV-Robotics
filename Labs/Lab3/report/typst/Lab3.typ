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
  show heading.where(level: 2): set block(above: 1.5em, below: 0.8em)
  
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
// (Empty in source)

/**************   Procedure   **************/
= Procedure

== Individual Work

=== Transformations in Practice

+ *MESSAGE VS. TF*
  - *Assume we have an incoming `geometry_msgs::Quaternion quat_msg` that holds the pose of our robot. We need to save it in an already defined `tf2::Quaternion quat_tf` for further calculations. Write one line of C++ code to accomplish this task.* \
    
    *Solution*: \
    To convert a `geometry_msgs::Quaternion` into a `tf2::Quaternion`, simply initialize the latter with the x, y, z, w components of the incoming message:
    
    ```cpp
    quat_tf = tf2::Quaternion(quat_msg.x, quat_msg.y, quat_msg.z, quat_msg.w);
    ```

    #figure(
      image("../source/img/tf2_Quaternion.png", width: 60%),
      caption: [tf2 Quaternion doc]
    ) <fig:tf2_doc>

  - *Assume we have just estimated our robot’s newest rotation and it’s saved in a variable called `quat_tf` of type `tf2::Quaternion`. Write one line of C++ code to convert it to a `geometry_msgs::Quaternion` type. Use `quat_msg` as the name of the new variable.* \
    
    *Solution*:
    ```cpp
    geometry_msgs::Quaternion quat_msg = tf2::toMsg(quat_tf);
    ```

  - If you just want to know the scalar value of a `tf2::Quaternion`, what member function will you use?

+ *CONVERSION*
  - Assume you have a `tf2::Quaternion quat_t`. How to extract the yaw component of the rotation with just one function call?
  
  - Assume you have a `geometry_msgs::Quaternion quat_msg`. How to you convert it to an Eigen 3-by-3 matrix? Refer to #link("https://docs.ros.org/en/jade/api/tf2_eigen/html/index.html")[this] for possible functions. You probably need two function calls for this.

=== Modelling and control of UAVs
+ STRUCTURE OF QUADROTORS
+ CONTROL OF QUADROTORS

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