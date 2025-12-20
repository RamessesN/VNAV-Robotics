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
  title: "Lab 6 Report",
  course: "Robotics Integration Group Project I",
  author-name: "Yuwei ZHAO",
  author-id: "23020036096",
  group: "Group #31",
  date: "2025-12-24",
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

=== Nister’s 5-point Algorithm

#boldify[
  Read the following paper. \
  [1] Nistér, David. “An efficient solution to the five-point relative pose problem.” 2003 IEEE Computer Society Conference on Computer Vision and Pattern Recognition, 2003. Vol. 2. 2003. link #link("https://www-users.cse.umn.edu/~hspark/CSci5980/nister.pdf")[here].

  Questions:
]

#boldify[
  1. Outline the main computational steps required to get the relative pose estimate (up to scale) in Nister’s 5-point algorithm.
]

#indent[
  *Step I --- Extract Nullspace Extraction:* \
  Build a $5 crossmark 9$ matrix, which consists of an epipolar constraint of 5 pairs of points, and calculate the zero-space vector $tilde(X), tilde(Y), tilde(Z), tilde(W)$ of the matrix. This step usually utilizes QR decomposition or SVD to achieve.

  *Step II --- Expand cubic constraints:* \
  Using the properties of the essential matrix, namely the trace constraint $2E E^T E - "trace"(E E^T)E = 0$. Express the eseential matrix as a linear combination of null space vectors $E = x X + y Y + z Z + omega W$, and then expand these cubic constraints.
  
  *Step III --- Gauss-Jordan Elimination:* \
  Build a $9 crossmark 20$ matrix $A$, and implement "Gauss-Jordan Elimination" to it.

  *Step IV --- Build & Solve Polynomial Formulation:* \
  Expand two $4 crossmark 4$ determinant polynomial of matrixes $B$ and $C$. Then get the 10#super("th") degree polynomial about the variable $z$ through elimination.

  *Step V --- Root Extraction:* \
  Solve for the real roots of this 10#super("th")-degree polynomial. This can be done by the Sturm sequence or by the eigenvalue decomposition of the adjoint matrix.

  *Step VI --- Recover Pose & Disambiguation:* \
  For each real root, the corresponding essential matrix $E$ is recovered, and then the rotation matrix $R$ and translation vector $t$ are decomposed. The points are triangulated using Cheirality constraint (i.e. points must be in front of the camera), and the only correct physical and geometric solution is selected from a maximum of 10 possible mathematical solutions.
]

#boldify[
  2. Does the 5-point algorithm exhibit any degeneracy? (degeneracy = special arrangements of the 3D points or the camera poses under which the algorithm fails)
]

#indent[
  _Conclusion_ --- In the main context discussed in the paper (i.e. compared to uncalibrated methods), the 5-point algorithm overcomes Planar Structure Degeneracy, which is one of the big advantages over 7 or 8-point algorithms.

  #line(length: 100%, stroke: (dash: "dashed"))

  *Explanation:*
  + *Non-degradation of planar scenes:* Section 4 of the paper makes it clear that for uncalibrated methods, the algorithm fails when the scene points are coplanar (there is a continuous solution). However, under the calibrated 5-point method setting, coplanar points lead to at most 2-fold ambiguity, which can usually be resolved by a third view, or excluded by chiral constraints. Therefore, the paper concludes that the algorithm still works well in planar scenes.

  + *General degradation:* While the paper focuses on its robustness in planar scenarios, according to the basics of epipolar geometry (and the implicit assumption in the paper), the essential matrix is not definable (baseline $t = 0$) without translational motion (pure rotation), at which point the algorithm degrades. In addition, multiple or infinite solutions also exist if 5 points and two optical centers lie on a specific critical surface such as twisted cubic, but it is considered robust for practical applications and planar scene comparisons highlighted in the paper.

]

#boldify[
  3. When used within RANSAC, what is the expected number of iterations the 5-point algorithm requires to find an outlier-free set?
]

#indent[
  From the standard theory of RANSAC, we have the expected number of iterations $N$ is determined by
  #set math.equation(numbering: none)
  $ N = log(1 - p) / log(1 - omega^s) $

  which #math.cases(
    $s = 5: "The minimum number of points required by the 5-point algorithm, i.e. sample size"$,
    $omega("Inlier Ratio"): "Proportion of inliers (i.e. the probability of selecting an inlier point in a sample)"$,
    $p("Confidence"): "The confidence that we want to find at least one full set of interior points"$
  )
  
  Since the term $omega^s$ decreases exponentially with $s$, a smaller sample size $s$ significantly reduces the required number of iterations. For the 5-point algorithm ($s=5$), the probability of selecting an outlier-free set is $omega^5$, which is much higher than for the 7-point ($s=7$) or 8-point ($s=8$) algorithms. This reduction is critical for real-time structure and motion estimation.

  #line(length: 100%, stroke: (dash: "dashed"))

  #strong[E.g.] Assuming a typical inlier ratio $omega = 0.5$ and a desired confidence $p = 0.99$, the required iterations $N$ are:

  #align(center)[
    #table(
      columns: (auto, auto, auto),
      inset: 1em,
      align: (col, row) => (left, center, left).at(col),
      table.header(
        [*Method*], [*Sample Size ($s$)*], [*Required Iterations ($N$)*]
      ),
      [5-point algorithm], [$5$], [$ceil(log(0.01) / log(1 - 0.5^5)) = 145$],
      [7-point algorithm], [$7$], [$ceil(log(0.01) / log(1 - 0.5^7)) = 588$],
      [8-point algorithm], [$8$], [$ceil(log(0.01) / log(1 - 0.5^8)) = 1177$],
    )
  ]
  
  Additionally, if we consider the strictly theoretical mean number of trials $E[k]$ required to encounter the first outlier-free set (geometric distribution expectation), it is given by:
  $ E[k] = 1 / omega^s $
  For $omega=0.5$, the 5-point algorithm requires on average $32$ trials, whereas the 8-point algorithm requires $256$ trials. This efficiency allows the algorithm to be executed for hundreds of samples within a small time budget.
]

=== Designing a Minimal Solver

#boldify[
  Can you do better than Nister? Nister’s method is a minimal solver since it uses 5 point correspondences to compute the 5 degrees of freedom that define the relative pose (up to scale) between the two cameras (recall: each point induces a scalar equation). In the presence of external information (e.g., data from other sensors), we may be able use less point correspondences to compute the relative pose.

  Consider a drone flying in an unknown environment, and equipped with a camera and an Inertial Measurement Unit (IMU). We want to use the feature correspondences extracted in the images captured at two consecutive time instants $t_1$ and $t_2$ to estimate the relative pose (up to scale) between the pose at time $t_1$ and the pose at time $t_2$. Besides the camera, we can use the IMU (and in particular the gyroscopes in the IMU) to estimate the relative rotation between the pose of the camera at time $t_1$ and $t_2$.

  You are required to solve the following problems:
]

#boldify[
  1. Assume the relative camera rotation between time and is known from the IMU. Design a minimal solver that computes the remaining degrees of freedom of the relative pose.
]

#indent[
  For the matching points $p_1, p_2$ on the normalized img plane, the epipolar constraint is \
  #set math.equation(numbering: none)
  $ p_2^T E p_1 = 0 $
  which essential matrix is $E = [t]#sub("x")R$. \
  Put $E$ into the known rotation, we have
  $ p_2^T ([t]#sub("x")R) p_1 = 0 $
  Let $p_1^' = R p_1$, then we have $p_2^T [t]#sub("x")p_1^' = 0 " ①"$ \
  $because$ ① is equivalent to scalar triple product \
  $therefore$ $t dot (p_1^' crossmark p_2) = 0$, which means the translation vector $t$ must be perpendicular to the vector $v = p_1^' crossmark p_2$ \
  $because$ 1 point can only ensure $t$ is on some plane \
  $therefore$ we need 2 points to ensure the direction of $t$

  #math.cases(
    $"For" 1#super("st") "pair points: normal vector" v_1 = (R p_(1, a)) crossmark p_(2, a)$,
    $"For" 2#super("nd") "pair points: normal vector" v_2 = (R p_(1, b)) crossmark p_(2, b)$
  )

  $because$ translation vector $t$ is perpendicular to $v_1, v_2$, thus
  $ t ~ v_1 crossmark v_2 $

  Then after normalizing, we have
  $ t arrow.l t / (||t||) $
]

#boldify[
  2. OPTIONAL (5 bonus pts): Describe the pseudo-code of a RANSAC algorithm using the minimal solver developed in point a) to compute the relative pose in presence of outliers (wrong correspondences).
]

#indent[
  #codly(languages: codly-languages)
  ```typc
  Algorithm: RANSAC for 2-point Relative Pose with Known Rotation

  Input: 
    Matches: A set of N feature correspondences S = {(p1_i, p2_i)}
    Rotation: Relative rotation matrix R (from IMU)
    Threshold: Inlier threshold epsilon (e.g., epipolar distance)
    Confidence: Desired confidence probability p (e.g., 0.99)
    InlierRatio: Estimated ratio of inliers w (e.g., 0.5)

  Output: 
    Best_t: The estimated translation direction
    Best_Inlier_Set: The set of consistent matches

  1. Max_Iterations = log(1 - p) / log(1 - w^2)
  2. Best_Score = 0
  3. Best_t = [0, 0, 0]

  4. For k = 1 to Max_Iterations:
      
      a. // Sampling
        Select 2 random correspondences {(p1_a, p2_a), (p1_b, p2_b)} from S.

      b. // Model Generation
        p1_prime_a = R * p1_a
        p1_prime_b = R * p1_b
        v1 = cross_product(p1_prime_a, p2_a)
        v2 = cross_product(p1_prime_b, p2_b)
        t_candidate = cross_product(v1, v2)
        Normalize t_candidate

      c. // Scoring (Count Inliers)
        Current_Score = 0
        Current_Inliers = {}
        E_candidate = [t_candidate]_cross * R  // Essential Matrix

        For each match (p1_i, p2_i) in S:
            // Calculate Sampson error or simple algebraic error
            error = abs( p2_i^T * E_candidate * p1_i ) 
            
            If error < epsilon:
                Current_Score = Current_Score + 1
                Add (p1_i, p2_i) to Current_Inliers

      d. // Update Best Model
        If Current_Score > Best_Score:
            Best_Score = Current_Score
            Best_t = t_candidate
            Best_Inlier_Set = Current_Inliers

  5. // Refinement
    Re-estimate t using all points in Best_Inlier_Set via Linear Least Squares or SVD.

  6. Return Best_t
  ```

  which
  #indent[
    - Sample Size --- It merely needs to sample 2 points, as opposed to 5 for the Nister algorithm, which means that the number of cycles of RANSAC will be greatly reduced ($N prop omega^(-2)$ vs $N prop omega^(-5)$), which greatly improves the computation speed.

    - Model Validation --- epipolar geometric error $p_2^T E p_1$ is used for validation for $R$ is known and $t$ is assumed so that $E$ is ensured.
  ]
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