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
  Based on the top-left block (Pose-Pose block) of the spy plot in `spy_game2.png`, there are **5** distinct diagonal blocks. Therefore, there are **5 robot poses**.
]

#boldify[
  2. How many landmarks exist in the map?
]

#indent[
  Based on the bottom-right block (Landmark-Landmark block), there are **6** distinct diagonal blocks. Therefore, there are **6 landmarks**.
]

#boldify[
  3. How many landmark have been observed by the current (last) pose?
]

#indent[
  The current pose corresponds to the last row (5th row) of the pose block. Looking at the off-diagonal block (top-right), there are **3** dark cells in this row. Thus, the current pose observed **3 landmarks** (specifically landmarks 4, 5, and 6).
]

#boldify[
  4. Which pose has observed the most number of landmark?
]

#indent[
  By counting the number of dark blocks in each row of the top-right quadrant:
  - Pose 1: 1 landmark
  - Pose 2: 2 landmarks
  - **Pose 3, Pose 4, and Pose 5**: Each observed **3 landmarks**.
  
  Therefore, **Poses 3, 4, and 5** tie for the most observations.
]

#boldify[
  5. What poses have observed the 2nd landmark?
]

#indent[
  Locating the 2nd column in the landmark section (top-right quadrant), there are dark blocks corresponding to **Pose 2 and Pose 3**. Thus, these two poses observed the 2nd landmark.
]

#boldify[
  6. Predict the sparsity pattern of the information matrix after marginalizing out the 2nd feature.
]

#indent[
  Marginalizing out the 2nd landmark will create a fill-in (dependency) between all poses that observed it. Since Pose 2 and Pose 3 observed Landmark 2, marginalizing it will strengthen the connection between **Pose 2 and Pose 3**. Since these poses are already sequentially connected ($P_{k-1}$ to $P_k$), the block structure will remain largely visually similar, but the density within the Pose 2-3 block will increase.
]

#boldify[
  7. Predict the sparsity pattern of the information matrix after marginalizing out past poses (i.e., only retaining the last pose).
]

#indent[
  Marginalizing out past poses (filtering approach) causes the information matrix to become **dense (fully connected)**. Eliminating a pose induces correlations between all landmarks observed by that pose and the subsequent pose. Repeating this process entangles all historic landmarks, destroying the sparse structure.
]

#boldify[
  8. Marginalizing out which variable (chosen among both poses or landmarks) would preserve the sparsity pattern of the information matrix?
]

#indent[
  Marginalizing out the **landmarks** (Schur complement on structure) preserves the sparsity pattern. This results in the "Reduced Camera System" matrix, which typically retains a **band-diagonal sparse structure**, as poses are only connected to other poses with which they share common landmark observations (usually temporal neighbors).
]

#boldify[
  9. The following figures illustrate the robot (poses-poses) block of the information matrix obtained after marginalizing out (eliminating) all landmarks in bundle adjustment in two different datasets. What can you say about these datasets (e.g., was robot exploring a large building? Or perhaps it was surveying a small room? etc) given the spy images below?

  #figure(
    image("img/spy_game1.png", width: 80%)
  )
]

#indent[
  - **Left Figure (Band-Diagonal):** The non-zero elements are concentrated strictly around the main diagonal. This indicates that the robot was likely **exploring a large environment (e.g., a long corridor) without revisiting** previous locations. Current poses only share information with spatially/temporally adjacent poses.
  
  - **Right Figure (Off-Diagonal / Loop Closures):** There are significant non-zero blocks far from the main diagonal (the "wings" in the corners). This indicates **Loop Closures**, where the current pose observes the same scene as a much earlier pose. This suggests the robot was likely **surveying a small room** or circling back to a previously visited area.
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

=== Prepare the Dataset

#indent[
  Download the datasets, then run them via the commands below:

  #align(center,
    math.cases(
      text(size: 1.5em, `./run_docker.sh orbslam:latest`),
      text(size: 1.5em, `./run_docker.sh kimera:latest`)
    )
  )

  Then, we can see the running results as follows:

  #figure(
    grid(
      columns: 2,
      gutter: 1em,
      image("img/orbslam.png"),
      image("img/kimera.png")
    ),
    caption: [*orbslam* & *kimera* Running Snapshot]
  )
]

=== Performance Comparison

#indent[
  After running the `fix_timestamps.py` to fix the timestamps of the trajectory files, then use `evo_traj` to compare *OrbSlam* and *Kimera*:
  
  `evo_traj tum output/kimera/kimera.txt output/orbslam/orb_slam3.txt --plot`

  And we have

  #figure(
    box(
      width: 85%,
      grid(
        columns: 2,
        gutter: 1em,

        image("img/ko_trajectories_without_align.PNG"),
        image("img/ko_xyz_without_align.PNG"),

        image("img/ko_rpy_without_align.PNG"),
        image("img/ko_speeds_without_align.PNG")
      )
    ),
    caption: [*OrbSlam / Kimera* (without aligning) Performance Comparison]
  )

  From the figures above, it can be seen that without alignment, the trajectories of *Kimera* (blue) and *OrbSlam3* (green) are spatially distinct, which is expected. The *3D Trajectory* and *XYZ* plots show different starting origins and orientations, indicating that each algorithm initialized its own local world coordinate frame at startup. The *RPY* plot confirms a significant constant offset in Yaw (approx. $150^"o"$ difference), while Roll and Pitch are more consistent due to gravity alignment. However, the *Speed* plot demonstrates high consistency in velocity estimation, proving that both algorithms are capturing the drone's dynamics correctly despite the coordinate frame mismatch.

  #line(length: 100%, stroke: (dash: "dashed"))

  After aligning the trajectories through `evo_traj euroc ~/datasets/vnav/MH_01_easy/mav0/state_groundtruth_estimate0/data.csv --save_as_tum` and then we run the comparison  `evo_traj tum output/kimera/kimera.txt output/orbslam/orb_slam3.txt --ref data.tum --plot --align` to compare them:

  #figure(
    box(
      width: 85%,
      grid(
        columns: 2,
        gutter: 1em,

        image("img/ko_trajectories.JPG"),
        image("img/ko_xyz.JPG"),

        image("img/ko_rpy.JPG"),
        image("img/ko_speeds.JPG")
      )
    ),
    caption: [*OrbSlam / Kimera* Performance Comparison]
  )

  From the aligned results above, it can be seen that after performing alignment against the Ground Truth, the estimated trajectories from both *Kimera* and *OrbSlam3* closely match the reference path. The *3D Trajectory* and *XYZ* plots show minimal drift, with both algorithms successfully tracking the complex motion of the MAV. The *RPY* plots indicate precise attitude estimation, accurately capturing rapid orientation changes. In the *Speed* plot, although both estimates contain typical high-frequency noise inherent to IMU-based prediction, they accurately follow the velocity profile of the ground truth. Overall, both systems demonstrate reliable state estimation performance on the EuRoC dataset.
]

=== LDSO

#indent[
  We successfully ran the LDSO (LiDAR-Direct-Sparse-Odometry) pipeline on the EuRoC dataset. As shown in the snapshot below, the viewer visualizes the sparse 3D point cloud reconstructed from the environment, along with the active keyframes and the current camera pose. The semi-dense reconstruction clearly outlines the structural features of the Machine Hall.

  #figure(
    image("./img/LDSO.png", width: 80%),
    caption: [*LDSO* Running Snapshot]
  )

  Since LDSO is a *monocular* direct visual odometry method, it inherently suffers from *scale ambiguity* (i.e., it cannot observe the absolute metric scale of the world). Therefore, when comparing its trajectory against the Ground Truth and stereo/VIO pipelines (Kimera and OrbSlam3), it is mandatory to use Sim3 alignment (alignment with rotation, translation, and *scale correction*). We enabled this using the `--correct_scale` flag in `evo`.

  #figure(
    box(
      width: 85%,
      grid(
        columns: 2,
        gutter: 1em,

        image("img/LDSO_trajectories.png"),
        image("img/LDSO_xyz.png"),

        image("img/LDSO_rpy.png"),
        image("img/LDSO_speeds.png")
      )
    ),
    caption: [*OrbSlam / Kimera / LDSO* Performance Comparison]
  )
  
  From the comparison plots above, several observations can be made:

  1. *Trajectories:* After applying scale correction, the LDSO trajectory (blue line, labeled `results_final`) aligns remarkably well with the Ground Truth (`data`), Kimera, and OrbSlam3. This demonstrates that despite lacking IMU data and stereo baselines, the direct photometric optimization of LDSO is capable of recovering the geometric structure of the camera path with high accuracy in feature-rich environments like EuRoC.

  2. *XYZ:* The LDSO trajectory appears quite smooth, particularly in the XYZ position plots. Direct methods often benefit from using information from all pixels with sufficient gradient, which can result in robust tracking even when specific corner features might be sparse, although it can be sensitive to photometric calibration and lighting changes.

  3. *RPY:* The Roll, Pitch, and Yaw estimates of LDSO track the ground truth variations accurately. However, unlike VIO methods (Kimera/OrbSlam3) which have an observable gravity vector from the accelerometer to constrain Roll and Pitch, monocular VO can sometimes exhibit slow drift in these axes over long durations. In this sequence, however, LDSO maintains its orientation stability effectively.

  4. *Speeds:* The velocity profile derived from the aligned LDSO trajectory matches the VIO pipelines. This confirms that the temporal consistency of the estimated pose is preserved, meaning the "virtual speed" in the scaled monocular frame corresponds linearly to the real-world speed after Sim3 alignment.
]

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