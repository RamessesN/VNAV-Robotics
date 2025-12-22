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

#let boldify(body) = {
  strong(body)
}

#let indent(body, amount: 1.2em) = {
  block(
    inset: (left: amount),
    above: 1em,
    below: 1em,
  )[
    #set par(leading: 1.0em)
    #body
  ]
}

#let sep = box(height: 1.5em)

#set math.cases(gap: 0.8em)

#let abstract(content) = {
  align(center, text(weight: "bold", size: 1.1em)[Abstract])
  pad(x: 2em, text(size: 0.9em, content))
}

#show: report-template.with(
  title: "Lab 7 Report",
  course: "Robotics Integration Group Project I",
  author-name: "Yuwei ZHAO",
  author-id: "23020036096",
  group: "Group #31",
  date: "2025-12-27",
)

/**************   Abstract   **************/
#abstract[
  ...

  See Resources on #link("https://github.com/RamessesN/Robotics_MIT")[github.com/RamessesN/Robotics_MIT].
]

/**************   Introduction   **************/
= Introduction

/**************   Procedure   **************/
= Procedure

/**************   Individual Work   **************/
== Individual Work

=== Spy Game

#boldify[
  Consider the following #link("https://www.mathworks.com/help/matlab/ref/spy.html")[spy]-style plot of an information matrix (i.e., coefficient matrix in Gauss-Newton’s normal equations) for a landmark-based SLAM problem where dark cells correspond to non-zero blocks:

  #figure(
    image("img/spy_game2.png", width: 40%)
  )

  Assuming robot poses are stored sequentially, answer the following questions:
]

#boldify[
  1. How many robot poses exist in this problem?
]

#indent[
  ...
]

#boldify[
  2. How many landmarks exist in the map?
]

#indent[
  ...
]

#boldify[
  3. How many landmark have been observed by the current (last) pose?
]

#indent[
  ...
]

#boldify[
  4. Which pose has observed the most number of landmark?
]

#indent[
  ...
]

#boldify[
  5. What poses have observed the 2nd landmark?
]

#indent[
  ...
]

#boldify[
  6. Predict the sparsity pattern of the information matrix after marginalizing out the 2nd feature.
]

#indent[
  ...
]

#boldify[
  7. Predict the sparsity pattern of the information matrix after marginalizing out past poses (i.e., only retaining the last pose).
]

#indent[
  ...
]

#boldify[
  8. Marginalizing out which variable (chosen among both poses or landmarks) would preserve the sparsity pattern of the information matrix?
]

#indent[
  ...
]

#boldify[
  9. The following figures illustrate the robot (poses-poses) block of the information matrix obtained after marginalizing out (eliminating) all landmarks in bundle adjustment in two different datasets. What can you say about these datasets (e.g., was robot exploring a large building? Or perhaps it was surveying a small room? etc) given the spy images below?

  #figure(
    image("img/spy_game1.png", width: 80%)
  )
]

#indent[
  ...
]

=== Well-begun is Half Done

#boldify[
  Pose graph optimization is a non-convex problem. Therefore, iterative solvers require a (good) initial guess to converge to the right solution. Typically, one initializes nonlinear solvers (e.g., Gauss-Newton) from the odometric estimate obtained by setting the first pose to the identity and chaining the odometric measurements in the pose graph.

  Considering that chaining more relative pose measurements (either odometry or loop closures) accumulates more noise (and provides worse initialization), propose a more accurate initialization method that also sets the first pose to the identity but chains measurements in the pose graph in a more effective way. A 1-sentence description and rationale for the proposed approach suffices.
]

#indent[
  ...
]

=== Feature-based methods for SLAM

#boldify[
  Read the ORB-SLAM paper (available #link("https://vnav.mit.edu/material/ORB-SLAM.pdf")[here]) and answer the following questions:
]

#boldify[
  1. Provide a 1 sentence description of each module used by ORB-SLAM (Fig. 1 in the paper can be a good starting point).
]

#indent[
  ...
]

#boldify[
  2. Consider the case in which the place recognition module provides an incorrect loop closure. How does ORB-SLAM check that each loop closure is correct? What happens if an incorrect loop closure is included in the pose-graph optimization module?
]

#indent[
  ...
]

=== Direct methods for SLAM

#boldify[
  Read the LSD-SLAM paper (available #link("https://vnav.mit.edu/material/LSD-SLAM.pdf")[here], see also the introduction below before reading the paper) and answer the following questions:
]

#boldify[
  1. Provide a 1 sentence description of each module used by LSD-SLAM and outline similarities and differences with respect to ORB-SLAM.
]

#indent[
  ...
]

#boldify[
  2. Which approach (between feature-based or direct) is expected to be more robust to changes in illumination or occlusions? Motivate your answer.
]

#indent[
  ...
]

=== From landmark-based SLAM to rotation estimation

#boldify[
  Consider the following landmark-based SLAM problem:

  #figure(
    image("img/formula.png", width: 90%)
  )

  Where the goal is to compute the poses of the robot $(t_i, R_i), i = 1, dots, N$ and the positions of point-landmarks $p_k, k = 1, dots, M$ given odometric measurements $(overline(t)_"ij", overline(R)_"ij")$ for each odometric edge $(i, j) in epsilon_0$ (here $epsilon_0$ denotes the set of odometric edges), and landmark observations $overline(p)_"ik"$ of landmark $k$ from pose $i$ for each observation edge $(i, k) in epsilon_l$ (here $epsilon_l$ denotes the set of pose-landmark edges).
]

#boldify[
  1. Prove the following claim: "The optimization problem (1) can be rewritten as a nonlinear optimization over the rotations $R_i, i = 1, dots, N$ only." Provide an expression of the resulting rotation-only problem to support the proof.
]

#indent[
  ...
]

#boldify[
  2. The elimination of variables discussed at the previous point largely reduces the size of the optimization problem (from 6N+3L variables to 3N variables).However, the rotation problem is not necessarily faster to solve. Discuss what can make the rotation-only problem more computationally-demanding to solve.
]

#indent[
  ...
]

/**************   Team Work   **************/
== Team Work

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