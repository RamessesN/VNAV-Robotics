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
  title: "Lab 4 Report",
  course: "Robotics Integration Group Project I",
  author-name: "Yuwei ZHAO",
  author-id: "23020036096",
  group: "Group #31",
  date: "2025-11-30",
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

=== Single-segment trajectory optimization

*Consider the following minimum velocity (r = 1) single-segment trajectory optimization problem:
*
*$ min_P(t)  integral_0^1 (P^(\(1\))(t))^2 d t $ <1>
s.t.
$ P(0) = 0, $ <2>
$ P(1) = 1, $ <3>*

*with $P(t) in RR[t]$, i.e., $P(t)$ is a polynomial function in $t$ with real coefficients:
$ P(t) = p_N t^N + p_(N-1) t^(N-1) + ... + p_1 t + p_0 $*

*Note that because of constraint (@2) $P(0) = p_0 = 0$, and we can parametrize $P(t)$ without a scalar part $p_0$.*

+ *Suppose we restrict $P(t) = p_1 t$ to be a polynomial of degree 1, what is the optimal solution of problem (@1)? What is the value of the cost function at the optimal solution?*

+ *Suppose now we allow $P(t)$ to have degree 2, i.e., $P(t) = p_2 t^2 + p_1 t$.*

  - *Write $integral_0^1 (P^(\(1\))(t))^2 d t$, the cost function of problem (@1), as $p^T Q p$, where $p = vec(p_1, p_2)$ and $Q in S^2$ is a symmetric $2 crossmark 2$ matrix.*

  - *Write $P(1) = 1$, constraint (@3), as $A p = b$, where $A in RR^(1 crossmark 2)$ and $b in RR$.*

  - *Solve the Quadratic Program (QP):
  $ min_p p^T Q p " " s.t. " " A p = b $ <5>*
  
  *You can solve it by hand, or you can solve it using numerical QP solvers (e.g., you can easily use the `quadprog` function in Matlab). What is the optimal solution you get for $P(t)$, and what is the value of the cost function at the optimal solution? Are you able to get a lower cost by allowing $P(t)$ to have degree 2?*

+ *Now suppose we allow $P(t) = p_3 t^3 + p_2 t^2 + p_1 t$:*
  - *Let $p = [p_1. p_2, p_3]^T$, write down $Q in S^3, A in RR^(1 crossmark 3), b in RR$ for QP (@5).*
  - *Solve the QP, what optimal solution do you get? Do this example agree with the result we learned from Euler-Lagrange equation in class?*

+ *Now suppose we are interested in adding one more constraint to problem (@1):
$ min_P(t)  integral_0^1 (P^(\(1\))(t))^2 d t, " " s.t. P(0) = 0, " " P(1) = 1, " " P^((1))(1) = -2 $ <6>
Using the QP method above, find the optimal solution and optimal cost of problem (@6) in the case of:*
  - *$P(t) = p_2 t^2 + p_1 t$, and*
  - *$P(t) = p_3 t^3 + p_2 t^2 + p_1 t$.*

=== Multi-segment trajectory optimization

+ *Assume our goal is to compute the minimum snap trajectory ($r = 4$) over $k$ segments. 
  How many and which type of constraints (at the intermediate points and at the start and end of the trajectory) do we need in order to solve this problem? 
  Specify the number of waypoint constraints, free derivative constraints and fixed derivative constraints.*

  #figure(
    image("./source/img/trajOpt.png", width: 60%)
  )

+ *Can you extend the previous question to the case in which the cost functional minimizes the $r$-th derivative and we have $k$ segments?*

== Team Work

=== Drone Racing

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