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
    From @3, we have: $Q = mat(1, 1, 1; 1, 4/3, 3/2; 1, 3/2, 9/5)$ #sep \
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