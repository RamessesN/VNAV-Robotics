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

#codly(languages: codly-languages)

#let abstract(content) = {
  align(center, text(weight: "bold", size: 1.1em)[Abstract])
  pad(x: 2em, text(size: 0.9em, content))
}

#show: report-template.with(
  title: "Lab 5 Report",
  course: "Robotics Integration Group Project I",
  author-name: "Yuwei ZHAO",
  author-id: "23020036096",
  group: "Group #31",
  date: "2025-12-20",
)

/**************   Abstract   **************/
#abstract[
  This report details the implementation and comparative analysis of a visual feature tracking, a critical component for Visual Odometry and SLAM systems. We developed a modular C++ framework to evaluate multiple feature descriptors --- *SIFT*, *SURF*, *ORB*, and *FAST+BRIEF* --- as well as the sparse Lucas-Kanade optical flow tracker. The pipeline includes feature detection, descriptor extraction, and correspondence establishment using Hamming distance. To ensure robust data association, we integrated outlier rejection mechanisms, specifically *Lowe’s Ratio Test* and Geometric Verification using *RANSAC-based* Fundamental Matrix estimation. 
  
  Experimental results on both static image pairs and real-world video sequences highlight the trade-offs between computational efficiency and invariance to viewpoint changes, demonstrating that while SIFT/SURF offer superior robustness to large rotations, binary descriptors and optical flow provide the real-time performance necessary for high-frequency robotic control.

  See Resources on #link("https://github.com/RamessesN/Robotics_MIT")[github.com/RamessesN/Robotics_MIT].
]

/**************   Introduction   **************/
= Introduction
  Visual feature tracking serves as the backbone for modern Visual Odometry (VO) and Simultaneous Localization and Mapping (SLAM) algorithms. The ability to robustly identify and track static landmarks across consecutive image frames allows a robot to estimate its ego-motion and reconstruct the 3D structure of its environment. However, this task is challenged by image noise, scale variations, illumination changes, and complex camera motions.

  In this lab, we address these challenges by implementing complete feature tracking algorithms. We begin by establishing the geometric foundations through the robust estimation of the Fundamental Matrix using RANSAC, which enforces the epipolar constraint to reject spurious matches. Subsequently, we extend the pipeline to support various feature extraction methodologies. We compare *Detect-and-Describe* approaches (SIFT, SURF, ORB, FAST+BRIEF) against *Sparse Optical Flow* (Lucas-Kanade), evaluating their performance in terms of inlier ratios, feature density, and computational cost.
  
  This comparative analysis provides a comprehensive understanding of how to select the appropriate tracking strategy based on specific application requirements, such as the need for rotation invariance versus the constraint of limited onboard computational resources.

/**************   Procedure   **************/
= Procedure

/**************   Individual Work   **************/
== Individual Work

=== Practice with Perspective Projection

#boldify[
  Consider a sphere with radius $r$ centered at $[0, 0, d]$ with respect to the camera coordinate frame (centered at the optical center and with axis oriented as discussed in class). Assume $d > r + 1$ and assume that the camera has principal point at (0, 0), focal length equal to 1, pixel sizes $s_x = s_y = 1$ and zero skew $s_theta = 0$ the following exercises:
]

#boldify[
  1. Derive the equation describing the projection of the sphere onto the image plane.
]

#indent[
  $because$ sphere radius is $r$ and its center is $[0, 0, d]$ \
  $therefore$ the equation of the sphere in the camera coordinate system is $X^2 + Y^2 + (Z - d)^2 = r^2$ \
  $because$ focal length equals 1, pixel sizes $s_x = s_y = 1$, principal point is (0, 0) and zero skew $s_theta = 0$ \
  $therefore$ camera intrinsic parameter matrix $K$ is an identity matrix. \
  
  Therefore, we have #math.cases(
    $u = (f dot x) / Z = x / Z$,
    $v = (f dot y) / Z = y / Z$
  ) \

  That's #math.cases(
    $x = u Z$,
    $y = v Z$
  ) \

  $therefore$ $(u Z)^2 + (v Z)^2 + (Z - d)^2 = r^2$ \
  $therefore$ $(u^2 + v^2 + 1) Z^2 - 2d Z + (d^2 - r^2) = 0$ \
  Let $Delta = b^2 - 4a c = (-2d)^2 - 4 (u^2 + v^2 + 1) (d^2 - r^2) = 0$ \
  $therefore$ $d^2 - (u^2 + v^2 + 1)(d^2 - r^2) = 0$ \
  $therefore$ $(u^2 + v^2)(d^2 - r^2) = r^2$ \
  That's $u^2 + v^2 = r^2 / (d^2 - r^2)$.

  #line(length: 100%, stroke: (dash: "dashed"))

  *Proof:* \
  $because$ $R_"img" = sqrt(r^2 / (d^2 - r^2)) = r / sqrt(d^2 - r^2)$
  
  and $tan^2 theta = sin^2 theta / (1 - sin^2 theta)$

  $therefore$ $u^2 = (r / d)^2 / (1 - (r / d)^2) = r^2 / d^2 - r^2$.

  #line(length: 100%, stroke: (dash: "dashed"))

  That's the equation of the projection of the sphere on the img plane is:
  #set math.equation(numbering: none)
  $ u^2 + v^2 = r^2 / (d^2 - r^2) $
 ]

#boldify[
  2. Discuss what the projection becomes when the center of the sphere is at an arbitrary location, not necessarily along the optical axis. What is the shape of the projection?
]

#indent[
  _Conclusion_ --- When the center of the sphere is located at an arbitrary position, the projected shape of the sphere on the image plane is usually an ellipse. The projection is a circle only if the center of the sphere lies exactly on the optical axis.

  #line(length: 100%, stroke: (dash: "dashed"))

  *Proof:* \

  Let #math.cases(
    $"Camera Center: " O(0, 0, 0)$,
    $"Sphere: radius-" r, "center-" c=[x_C, y_C, z_C]^T$,
    $"Point: " x=[u, v, 1]^T ("WLOG" f = 1)$
  ) \

  $therefore$ we have $sin(theta) = r / (||theta||) => cos^2 theta = 1 - sin^2 theta = (||c||^2 - r^2) / (||c||^2)$ \
  $because$ $x dot c = ||x|| ||c|| cos(theta)$ \
  $therefore$ $(x dot c)^2 = ||x||^2 ||c||^2 cos^2(theta) = ||x||^2 (||c||^2 - r^2)$ \
  Let $Kappa = ||c||^2 - r^2$, we have $(x dot c)^2 = Kappa ||x||^2$ \
  Put $x = [u, v, 1]^T$ into the equation, \
  $=>$ $(x_C u + y_C v + z_C)^2 = Kappa (u^2 + v^2 + 1)$ \

  $therefore$ #math.cases(
    $(x_C u + y_C v + z_C)^2 = x_C^2 u^2 + y_C^2 v^2 + z_C^2 + 2 x_C y_C u v + 2 x_C z_C u + 2 y_C z_C v$,
    $Kappa (u^2 + v^2 + 1) = Kappa u^2 + Kappa v^2 + Kappa$
  ) \

  $therefore$ $(Kappa - x_C^2) u^2 - 2 x_C y_C u v + (Kappa - y_C^2) v^2 + dots = 0$ \
  That's the shape of the projection is a *ellipse*.
]

=== Vanishing Points

#boldify[
  Consider two 3D lines that are parallel to each other. As we have seen in the lectures, lines that are parallel in 3D may project to intersecting lines on the image plane. The pixel at which two 3D parallel lines intersect in the image plane is called a vanishing point. Assume a camera with principal point at (0,0), focal length equal to 1, pixel sizes $s_x = s_y = 1$ and zero skew $s_theta = 0$. Complete the following exercises:
]

#boldify[
  1. Derive the generic expression of the vanishing point corresponding to two parallel 3D lines.
]

#indent[
  We have $p(lambda) = p_0 + lambda d$, which \
  #math.cases(
    $p(lambda) = [X(lambda), Y(lambda), Z(lambda)]^T "are the points on the line"$,
    $p_0 = [X_0, Y_0, Z_0]^T "is the start point on the line"$,
    $d = [d_x, d_y, d_z]^T "is the unit vector of the line"$,
    $lambda in RR "is a parameter"$
  )

  $therefore$ #math.cases(
    $X(lambda) = X_0 + lambda d_x$,
    $Y(lambda) = Y_0 + lambda d_y$,
    $Z(lambda) = Z_0 + lambda d_z$
  )

  $because$ $u_"img" = X / Z, v_"img" = Y / Z$
  
  $therefore$ #math.cases(
    $u_"img"(lambda) = (X_0 + lambda d_x) / (Z_0 + lambda d_z)$,
    $v_"img"(lambda) = (Y_0 + lambda d_y) / (Z_0 + lambda d_z)$
  )

  $therefore$ $lim_(lambda -> infinity) u_"img" (lambda) = lim_(lambda -> infinity) (X_0 + lambda d_x) / (Z_0 + lambda d_z) = lim_(lambda -> infinity) (X_0 / lambda + d_x) / (Z_0 / lambda + d_z) = d_x / d_z$

  Similarly, we have $lim_(lambda -> infinity) v_"img" = lim_(lambda -> infinity) (Y_0 + lambda d_y) / (Z_0 + lambda d_z) = d_y / d_z$

  That's *Vanishing Point* is $(d_x / d_z, d_y / d_z)$.
]

#boldify[
  2. Find (and prove mathematically) a condition under which 3D parallel lines remain parallel in the image plane.
]

#indent[
  _Conclusion_ --- If two 3D lines are still parallel on the img plane, that means they don't have a finite coordinate intersection 
  (i.e., they don't intersect, or they intersect at infinity). This happens when "denominator = 0", i.e. \
  #set math.equation(numbering: none)
  $ d_z = 0 $
  
  #line(length: 100%, stroke: (dash: "dashed"))

  *Proof:* \
  WLOG there are two 3D parallel lines: $L_1, L_2$ \
  which have the shared unit vector $d = [d_x, d_y, 0]^T$ but go through different points $p_1, p_2$ \
  *For Line $L_1$:* origin $p_1 = [X_1, Y_1, Z_1]^T$

  $therefore$ we have #math.cases(
    $X(lambda) = X_1 + lambda d_x$,
    $Y(lambda) = Y_1 + lambda d_y$,
    $Z(lambda) = Z_1 + lambda dot 0 = Z_1"(which Z is a constant)"$
  )

  Project onto the image plane $(u, v)$, we have #math.cases(
    $u = (X_1 + lambda d_x) / Z_1 = X_1 / Z_1 + lambda d_x / Z_1"   "①$,
    $v = (Y_1 + lambda d_y) / Z_1 = Y_1 / Z_1 + lambda d_y / Z_1"     "②$
  )

  In order to find the line equation on the img plane(remove $lambda$), assume that $d_x eq.not 0$, we have \
  $lambda = (Z_1 u - X_1) / d_x arrow.l ①$

  Put into $②$, we have 
  $v = Y_1 / Z_1 + d_y / Z_1 ((Z_1 u - X_1) / d_x)$

  $therefore$ $v = (d_y / d_x)u + (Y_1 / Z_1 - (X_1 d_y) / (Z_1 d_x))$

  This is a "$v = m u + c$" -format line equation, which slope $m_1 = d_y / d_x$

  *For Line $L_2$:* origin $p_2 = [X_2, Y_2, Z_2]^T$, unit vector is the same --- $d$ \ 
  Similarly, we have the line equation: \
  $v = (d_y / d_x) u + (Y_2 / Z_2 - (X_2 d_y) / (Z_2 d_x))$, which slope $m_2 = d_y / d_x$

  $because$ $m_1 = m_2 = d_y / d_x$ $=>$ The two lines on the img plane have the same slope \
  *Therefore*, when 3D lines are parallel to the img plane(i.e. $d_z = 0$), their projections remain plane on the img plane.
]

/**************   Team Work   **************/
== Team Work

=== Feature Descriptors (SIFT)

#indent[
  In this section, our primary objective is to implement the front-end of a feature-based visual odometry pipeline. The goal is to establish reliable data association between consecutive frames by detecting and matching distinctive visual features. Specifically, we utilize the *Scale-Invariant Feature Transform (SIFT)* for its robustness to scale and rotation changes. The implementation is structured to separate the specific feature algorithm from the general tracking pipeline, ensuring modularity for subsequent extensions.

  #line(length: 100%, stroke: (dash: "dashed"))

  #strong[For #link(<section:sift_feature_tracker>)[`sift_feature_tracker.cpp`]:]

  In this file, we implemented the specific logic for the SIFT (Scale-Invariant Feature Transform) algorithm.
  
  For `detectKeypoints` and `describeKeypoints`, we utilized the OpenCV `SIFT::create()` interface. SIFT detects keypoints by searching for local extrema in the Difference of Gaussians (DoG) scale space, ensuring scale invariance. The descriptors are then constructed as 128-dimensional vectors based on the gradient magnitudes and orientations in the local neighborhood, providing rotation invariance.

  #line(length: 100%, stroke: (dash: "dashed"))

  #strong[For #link(<section:feature_tracker>)[`feature_tracker.cpp`]:]
  
  We modified the `trackFeatures` function to orchestrate the pipeline. In the detection phase, we call `detectKeypoints` on both input images. Then with the command `roslaunch lab_5 two_frames_tracking.launch descriptor:=SIFT`, we obtain the visualization of local features.

  As shown in the figure below, the keypoints (visualized with size and orientation) are densely distributed on high-contrast areas, such as the text and edges of the box, validating the effectiveness of the detector.

  #figure(
    grid(
      columns: (1fr, 1fr),
      gutter: -3em,
      image("img/sift_keypoints_1.jpg", height: 17%),
      image("img/sift_keypoints_2.jpg", height: 17%),
    ),
    caption: [Local Feature Extraction (SIFT Keypoints)],
  )
]

=== Descriptor-based Feature Matching

#indent[
  In this section, we focus on establishing correspondences between the features detected in the previous step. We implemented the `matchDescriptors` function in `sift_feature_tracker.cpp`.
  
  Instead of simple brute-force matching, which can be computationally expensive, we employed a *FLANN-based matcher* with K-Nearest Neighbors (KNN, $k=2$). Crucially, to filter out ambiguous matches, we applied *Lowe's Ratio Test*.
  
  The mathematical condition for this test is:
  #set math.equation(numbering: none)
  $ d_1 < "ratio" dot d_2 $
  
  where $d_1$ and $d_2$ are the Euclidean distances to the closest and the second-closest descriptors, respectively. We set the threshold `ratio_thresh` to 0.8. This strategy rejects keypoints that are not discriminative enough (e.g., repetitive patterns on the box surface), significantly improving the matching precision.

  The result of descriptor-based matching is shown in the figure below. While most matches correctly link the semantic parts of the box (e.g., corners to corners), there are still visible **outliers** (crossing lines) that violate the motion consistency. These outliers indicate that appearance-based matching alone is insufficient.

  #figure(
    image("img/sift_matches.jpg", width: 80%),
    caption: [Descriptor-based Feature Matching (Filtered by Ratio Test)],
  )
]

=== Keypoint Matching Quality

#indent[
  To further improve the quality of correspondences, we implemented Geometric Verification in `feature_tracker.cpp` using the `inlierMaskComputation` function.
  
  We utilize the *RANSAC (Random Sample Consensus)* algorithm to estimate the Fundamental Matrix ($F$) that best fits the matches obtained from the previous step. The underlying mathematical principle is the *Epipolar Constraint*:
  
  #set math.equation(numbering: none)
  $ p_2^T F p_1 = 0 $
  
  where $p_1$ and $p_2$ are the homogeneous coordinates of matched points in the first and second images. Matches that do not satisfy this geometric constraint (within a threshold of 3.0 pixels) are classified as outliers and removed.

  #figure(
    image("img/sift_inliers.PNG", width: 80%),
    caption: [Inliers after RANSAC Geometric Verification],
  )

  Comparing the figure above with the result in Deliverable 4, the erroneous "crossing lines" have been successfully removed. The remaining matches (Inliers) show a consistent flow field, representing the true motion of the camera relative to the box.

  The metric print part in terminal log is as follows:
  ```log
  Avg. Keypoints 1 Size: 603
  Avg. Keypoints 2 Size: 969
  Avg. Number of matches: 603
  Avg. Number of good matches: 98
  Avg. Number of Inliers: 77
  Avg. Inliers ratio: 0.785714
  Num. of samples: 1
  ```

  From the terminal log, the quantitative statistics of the matching process are summarized in the table below:

  #figure(
    table(
      columns: (auto, auto),
      inset: 5pt,
      align: center + horizon,
      [*Metric*], [*Value*],
      [\# of Keypoints in Img 1], [603],
      [\# of Keypoints in Img 2], [969],
      [\# of Matches], [603],
      [\# of Good Matches], [98],
      [\# of Inliers], [77],
      [Inlier Ratio], [78.6%],
    ),
    caption: [Matching Statistics for SIFT Descriptor],
  )
]

=== Comparing Feature Matching Algorithms on Real Data

#indent[
  In this section, we extend our feature tracking framework to evaluate three additional popular algorithms: _SIFT_, _SURF_, _ORB_, and _FAST_ (coupled with _BRIEF_ descriptors). The objective is to perform a comprehensive comparison of these methods against the SIFT baseline implemented in previous sections.
  
  We implemented the corresponding derived classes (`SurfFeatureTracker`, `OrbFeatureTracker`, and `FastFeatureTracker`) in C++. A key implementation detail is the adjustment of the matching strategy: while floating-point descriptors (SIFT/SURF) use the Euclidean distance ($L_2$ norm), binary descriptors (ORB/BRIEF) are matched using the *Hamming distance* to ensure efficiency and correctness.
  
  The evaluation is conducted in two distinct scenarios to assess different performance characteristics:
  1. *Pair of Frames:* To test robustness against large viewpoint changes and rotations.
  2. *Real Datasets (Video):* To evaluate stability and consistency in a continuous, small-baseline visual odometry setting.
  
  #strong[6.a. Pair of frames]

  #figure(
    image("img/demonstration_frame.png", width: 70%),
    caption: [Video Stream Processing Demonstration],
  )

  Run the following command four times(SIFT, SURF, ORB, and FAST+BRIEF)

  `roslaunch lab_5 two_frames_tracking.launch descriptor:=<NAME_OF_DOWNLOADED_FILE>`
  
  Then, we have

  #grid(
    columns: (1fr, 1fr),
    gutter: 1em,

    figure(
      ```log
      Avg. Keypoints 1 Size: 603
      Avg. Keypoints 2 Size: 969
      Avg. Number of matches: 603
      Avg. Number of good matches: 98
      Avg. Number of Inliers: 77
      Avg. Inliers ratio: 0.785714
      Num. of samples: 1
      ```,
      caption: [SIFT Log],
      kind: "code",
      supplement: [Log],
    ),
    figure(
      ```log
      Avg. Keypoints 1 Size: 997
      Avg. Keypoints 2 Size: 1822
      Avg. Number of matches: 997
      Avg. Number of good matches: 135
      Avg. Number of Inliers: 90
      Avg. Inliers ratio: 0.666667
      Num. of samples: 1
      ```,
      caption: [SURF Log],
      kind: "code",
      supplement: [Log],
    ),

    figure(
      ```log
      Avg. Keypoints 1 Size: 500
      Avg. Keypoints 2 Size: 500
      Avg. Number of matches: 500
      Avg. Number of good matches: 8
      Avg. Number of Inliers: 7
      Avg. Inliers ratio: 0.875
      Num. of samples: 1
      ```,
      caption: [ORB Log],
      kind: "code",
      supplement: [Log],
    ),
    figure(
      ```log
      Avg. Keypoints 1 Size: 2266
      Avg. Keypoints 2 Size: 3653
      Avg. Number of matches: 2266
      Avg. Number of good matches: 41
      Avg. Number of Inliers: 17
      Avg. Inliers ratio: 0.414634
      Num. of samples: 1
      ```,
      caption: [FAST+BRIEF Log],
      kind: "code",
      supplement: [Log],
    ),
  )

  From the four terminal logs above, we can fill the metrics-comparison table as follows:

  #figure(
    table(
      columns: (auto, auto, auto, auto, auto),
      inset: 5pt,
      align: center + horizon,
      table.cell(
        colspan: 1,
        rowspan: 2,
        [*Statistics*],
      ),
      table.cell(
        colspan: 4,
        [*Approach for Dataset X*],
      ),

      [*SIFT*], [*SURF*], [*ORB*], [*FAST+BRIEF*],
      [*\# of Keypoints in Img 1*], [603], [997], [500], [2266],
      [*\# of Keypoints in Img 2*], [969], [1822], [500], [3653],
      [*\# of Matches*], [603], [997], [500], [2266],
      [*\# of Good Matches*], [98], [135], [8], [41],
      [*\# of Inliers*], [77], [90], [7], [17],
      [*Inlier Ratio*], [78.6%], [66.7%], [87.5%], [41.5%],
    ),
    caption: [Performance Comparison of Different Feature Trackers for Pair of Frames],
  )

  #strong[Analysis for Pair of Frames:]
  #indent[
  1.  *Floating-point Descriptors (SIFT/SURF):* Both SIFT and SURF demonstrated high robustness to the large viewpoint change (rotation and scale) between the two static images. SIFT achieved a solid 78.6% inlier ratio, confirming its scale and rotation invariance properties. SURF detected more keypoints (1822 vs 969) but had a slightly lower inlier ratio (66.7%).

  2.  *Binary Descriptors (ORB/FAST+BRIEF):* The binary descriptors struggled with the large baseline matching task compared to SIFT.
    - *FAST+BRIEF* detected a massive number of keypoints (3653) but suffered from a low inlier ratio (41.5%). This is expected because the standard BRIEF descriptor is not invariant to large rotations, which are present in the image pair.
    - *ORB* showed an anomaly with very few surviving good matches (only 8). This is likely due to the strictness of *Lowe's Ratio Test (0.8)* when applied to Hamming distances on a texture that isn't ideal for corner detection. However, the few matches that survived were highly accurate (87.5% inlier ratio).
  ]
  #strong[6.b. Real Datasets]

  Run the following command four times(SIFT, SURF, ORB, and FAST+BRIEF) for two datasets:
  
   `roslaunch lab_5 video_tracking.launch path_to_dataset:=/home/$USER/Desktop/<NAME_OF_DOWNLOADED_FILE>.bag descriptor:=<NAME_OF_DOWNLOADED_FILE>` 

  #strong[$section$ `30fps_424x240_2018-10-01-18-35-06.bag`]

  #figure(
    image("img/demonstration_video_30fps.png", width: 70%),
    caption: [Video Stream Processing Demonstration],
  )

  #grid(
    columns: (1fr, 1fr),
    gutter: 1em,

    figure(
      ```log
      Avg. Keypoints 1 Size: 321.819
      Avg. Keypoints 2 Size: 321.602
      Avg. Number of matches: 321.819
      Avg. Number of good matches: 160.974
      Avg. Number of Inliers: 149.751
      Avg. Inliers ratio: 0.928412
      Num. of samples: 425
      ```,
      caption: [SIFT Log],
      kind: "code",
      supplement: [Log],
    ),
    figure(
      ```log
      Avg. Keypoints 1 Size: 389.408
      Avg. Keypoints 2 Size: 389.252
      Avg. Number of matches: 389.408
      Avg. Number of good matches: 234.594
      Avg. Number of Inliers: 216.93
      Avg. Inliers ratio: 0.926189
      Num. of samples: 488
      ```,
      caption: [SURF Log],
      kind: "code",
      supplement: [Log],
    ),

    figure(
      ```log
      Avg. Keypoints 1 Size: 268.751
      Avg. Keypoints 2 Size: 268.612
      Avg. Number of matches: 268.751
      Avg. Number of good matches: 166.031
      Avg. Number of Inliers: 160.54
      Avg. Inliers ratio: 0.964453
      Num. of samples: 541
      ```,
      caption: [ORB Log],
      kind: "code",
      supplement: [Log],
    ),
    figure(
      ```log
      Avg. Keypoints 1 Size: 741.136
      Avg. Keypoints 2 Size: 741.256
      Avg. Number of matches: 741.136
      Avg. Number of good matches: 499.004
      Avg. Number of Inliers: 465.635
      Avg. Inliers ratio: 0.928044
      Num. of samples: 477
      ```,
      caption: [FAST+BRIEF Log],
      kind: "code",
      supplement: [Log],
    ),
  )

  From the four terminal logs above we can fill the metrics-comparison table as follows:
  
  #figure(
    table(
      columns: (auto, auto, auto, auto, auto),
      inset: 5pt,
      align: center + horizon,
      table.cell(
        colspan: 1,
        rowspan: 2,
        [*Statistics*],
      ),
      table.cell(
        colspan: 4,
        [*Approach for Dataset X*],
      ),

      [*SIFT*], [*SURF*], [*ORB*], [*FAST+BRIEF*],
      [*\# of Keypoints in Img 1*], [321.819], [389.408], [268.751], [741.136],
      [*\# of Keypoints in Img 2*], [321.602], [389.252], [268.612], [741.256],
      [*\# of Matches*], [321.819], [389.408], [268.751], [741.136],
      [*\# of Good Matches*], [160.974], [234.594], [166.031], [499.004],
      [*\# of Inliers*], [149.751], [216.93], [160.54], [465.635],
      [*Inlier Ratio*], [92.8%], [92.6%], [96.4%], [92.8%],
    ),
    caption: [Performance Comparison of Different Feature Trackers for Real Datasets],
  )

  #strong[$section$ `vnav-lab5-smooth-trajectory.bag`]

  #figure(
    image("img/demonstration_video_vnav.png", width: 70%),
    caption: [Video Stream Processing Demonstration],
  )

  #grid(
    columns: (1fr, 1fr),
    gutter: 1em,

    figure(
      ```log
      Avg. Keypoints 1 Size: 137.468
      Avg. Keypoints 2 Size: 136.608
      Avg. Number of matches: 137.468
      Avg. Number of good matches: 67.9915
      Avg. Number of Inliers: 50.8766
      Avg. Inliers ratio: 0.75093
      Num. of samples: 235
      ```,
      caption: [SIFT Log],
      kind: "code",
      supplement: [Log],
    ),
    figure(
      ```log
      Avg. Keypoints 1 Size: 1009.98
      Avg. Keypoints 2 Size: 1006.7
      Avg. Number of matches: 1009.98
      Avg. Number of good matches: 457.648
      Avg. Number of Inliers: 345.381
      Avg. Inliers ratio: 0.778896
      Num. of samples: 310
      ```,
      caption: [SURF Log],
      kind: "code",
      supplement: [Log],
    ),

    figure(
      ```log
      Avg. Keypoints 1 Size: 258.294
      Avg. Keypoints 2 Size: 257.972
      Avg. Number of matches: 258.294
      Avg. Number of good matches: 133.933
      Avg. Number of Inliers: 116.456
      Avg. Inliers ratio: 0.878598
      Num. of samples: 465
      ```,
      caption: [ORB Log],
      kind: "code",
      supplement: [Log],
    ),
    figure(
      ```log
      Avg. Keypoints 1 Size: 524.664
      Avg. Keypoints 2 Size: 523.742
      Avg. Number of matches: 524.664
      Avg. Number of good matches: 313.428
      Avg. Number of Inliers: 273.366
      Avg. Inliers ratio: 0.877645
      Num. of samples: 465
      ```,
      caption: [FAST+BRIEF Log],
      kind: "code",
      supplement: [Log],
    ),
  )

  From the four terminal logs above, we can fill the metrics-comparison table as follows:

  #figure(
    table(
      columns: (auto, auto, auto, auto, auto),
      inset: 5pt,
      align: center + horizon,
      table.cell(
        colspan: 1,
        rowspan: 2,
        [*Statistics*],
      ),
      table.cell(
        colspan: 4,
        [*Approach for Dataset X*],
      ),

      [*SIFT*], [*SURF*], [*ORB*], [*FAST+BRIEF*],
      [*\# of Keypoints in Img 1*], [137.468], [1009.98], [258.294], [524.664],
      [*\# of Keypoints in Img 2*], [136.608], [1006.7], [257.972], [523.742],
      [*\# of Matches*], [137.468], [1009.98], [258.294], [524.664],
      [*\# of Good Matches*], [67.9915], [457.648], [133.933], [313.428],
      [*\# of Inliers*], [50.8766], [345.381], [116.456], [273.366],
      [*Inlier Ratio*], [75.1%], [77.9%], [87.9%], [87.8%],
    ),
    caption: [Performance Comparison of Different Feature Trackers for Real Datasets],
  )

  #strong[Analysis for Real Datasets (Video Sequence):]
  #indent[
    1. *High Precision of Binary Descriptors:* In this video sequence, the binary descriptors (*ORB* and *FAST+BRIEF*) outperformed the floating-point descriptors in terms of matching precision, both achieving inlier ratios of approximately *88%*. This demonstrates that for smooth, small-baseline motion, binary descriptors can provide highly reliable data association with fewer outliers than SIFT or SURF (which ranged between 75-78%).

    2. *Feature Density Trade-off:*
      - *SURF* detected the highest volume of features (over 1000 raw keypoints) and maintained the highest number of inliers (345), providing the densest tracking field. This is beneficial for robust pose estimation but comes at a high computational cost.
      - *SIFT* generated a relatively sparse map (only ~50 inliers), which might be risky for continuous tracking if texture becomes scarce.
      - *FAST+BRIEF* offered a good balance, providing the second-highest density (273 inliers) while maintaining high accuracy.

    3. *Computational Efficiency:* We observed distinct differences in processing speed. *ORB* and *FAST* processed frames in real-time, making them ideal for onboard robot navigation. In contrast, *SURF*, despite its high feature density, showed noticeable latency due to the heavy computational load of extracting and matching over 1000 floating-point descriptors per frame.
  ]

  #strong[Conclusion:]
  #indent[
    - For applications requiring invariance to large scale/rotation changes (e.g. Loop Closure or Wide-baseline Stereo), *SIFT* or *SURF* is preferred despite the computational cost.
    - For real-time Visual Odometry or SLAM with high frame rates (small baselines), *ORB* or *FAST+BRIEF* offers the best trade-off between speed and accuracy.
  ]
]

=== Feature Tracking: Lucas Kanade tracker

#indent[
  In this final deliverable, we implemented the Lucas-Kanade (LK) tracker, which represents a fundamentally different approach compared to the descriptor-based methods (SIFT, SURF, ORB) evaluated previously.
  
  Instead of detecting features independently in each frame and matching them based on descriptor similarity, the LK tracker relies on the _Brightness Constancy Assumption_:
  #set math.equation(numbering: none)
  $ I(x, y, t) approx I(x+d x, y+d y, t+d t) $
  It solves for the motion vector $(u, v)$ by minimizing the error in a local window, utilizing the image spatial gradients. We implemented this using `cv::calcOpticalFlowPyrLK` with a pyramidal approach to handle larger displacements. The tracking is initialized using *Good Features to Track (GFTT)* in the first frame.

  We evaluated the LK tracker on the two real-world datasets. The code is #link(<section:lk_feature_tracker>)[here] and the visual results and statistical logs are presented below:

  #figure(
    grid(
      columns: 2,
      gutter: 1em,
      image("img/LK_30fps.png"),
      image("img/LK_vnav.png"),
    ),
    caption: [Lucas Kanade for Real Datasets]
  )

  #strong[$section$ `30fps_424x240_2018-10-01-18-35-06.bag`]

  ```log
  [INFO] [1766307648.340722934]: LK Stats: Matches: 776 Inliers: 756 Ratio: 0.974227
  ```

  #strong[$section$ `vnav-lab5-smooth-trajectory.bag`]

  ```log
  [INFO] [1766308353.398518896]: LK Stats: Matches: 653 Inliers: 634 Ratio: 0.970904
  ```

  #line(length: 100%, stroke: (dash: "dashed"))

  #strong[Analysis and Observations:]

  Based on the logs above and the comparison with previous descriptor-based methods, we observe the following:
  #indent[
    1. _Extremely High Inlier Ratio (>97%)_: The LK tracker achieved the highest inlier ratio (~97.4%) among all tested algorithms.
      - _Reason_: Descriptor matching (e.g. SIFT/ORB) performs a global search, which can lead to "outliers" where a feature in the top-left corner matches a similar-looking feature in the bottom-right.
      - _Contrast_: LK performs a local search. It assumes the feature in the next frame is close to its previous position. This constraint naturally filters out global gross outliers, resulting in exceptionally clean data association for smooth video sequences.

    2. _Efficiency and Speed_: Although precise timing logs are not shown here, the LK method skips the computationally expensive "Descriptor Computation" and "Brute-force Matching" steps. It only requires computing image gradients, making it significantly faster and highly suitable for high-frequency control loops (e.g. drone hovering).
    
    3. _Assumption Dependencies_: The LK tracker performed excellently here because the datasets (30fps video) satisfy the *Small Motion Assumption*. If the camera were to move very rapidly or if frames were dropped, the local window search would fail, and the tracker would lose features. In contrast, SIFT/ORB would be more robust to such large, discontinuous jumps.
  ]

  #strong[Conclusion:] 
  
  The Lucas-Kanade tracker provides the most precise and efficient tracking for *continuous*, *smooth video streams*. However, it lacks the ability to recover from lost tracks (Relocalization) or handle wide-baseline matching, for which descriptor-based methods (like ORB or SIFT) are required.
]

=== Optical Flow

#indent[
  #figure(
    grid(
      columns: 2,
      gutter: 1em,
      image("img/farneback_30fps.png", height: 20%),
      image("img/farneback_vnav.png", height: 20%),
    ),
    caption: [Self Optical Flow --- Farneback Algorithm]
  )

  Unlike the sparse feature tracking methods (SIFT/LK) evaluated previously, the Farneback algorithm provides a *dense estimation* of the optical flow field. The detailed code is #link(<section:self_flow>)[here]. As shown in the Figure, we visualized the flow using the HSV color space, where *Hue* represents the motion direction and *Value* represents the velocity magnitude.

  *Comparison with LK Tracker*: While LK provides efficient tracking of specific keypoints suitable for pose estimation (SLAM), Farneback provides motion vectors for every pixel. This allows for a richer understanding of the scene structure and moving objects (e.g., we can clearly see the shape of the moving box). However, this comes at a significant computational cost. We observed that the dense flow calculation introduced noticeable latency compared to the real-time performance of the LK tracker.
]

/**************   Reflection and Analysis   **************/
= Reflection and Analysis
#indent[
  Throughout this lab, we have implemented and evaluated the complete front-end of a visual navigation pipeline. By experimenting with various feature descriptors (_SIFT_, _SURF_, _ORB_, and _FAST+BRIEF_) and tracking paradigms (Descriptor Matching vs. Lucas-Kanade Optical Flow), we gained critical insights into the trade-offs inherent in visual odometry systems.

  #strong[1. The Trade-off between Invariance and Efficiency]
  
  Our experiments in Deliverable 6 revealed a fundamental dichotomy between floating-point and binary descriptors:
  - *Floating-point descriptors (SIFT/SURF)* demonstrated superior robustness to large scale and rotation changes (as seen in the "Pair of Frames" experiment). They are ideal for tasks like Loop Closure Detection or 3D Reconstruction where wide-baseline matching is required. However, their computational cost (high-dimensional vector calculation) makes them less suitable for high-speed, resource-constrained platforms.
  - *Binary descriptors (ORB/FAST+BRIEF)* and *Optical Flow (LK)* excelled in computational efficiency, achieving real-time performance on video sequences. However, they proved fragile under large viewpoint changes (e.g., ORB failed to match the rotated box in Deliverable 6.a). This restricts their use to small-baseline tracking scenarios, such as high-frame-rate Visual Odometry (VO).

  #strong[2. The Critical Role of Geometric Verification]
  
  In the individual work and Deliverable 5, we observed that appearance-based matching alone is insufficient. Raw feature matches often contain outliers due to repetitive textures (e.g., identical letters on the box).
  The integration of *RANSAC* with the *Epipolar Constraint (Fundamental Matrix)* was the turning point. It filtered out matches that were visually similar but geometrically impossible, increasing the reliability of the system. This highlights that a robust VIO frontend must rely on both photometric consistency (descriptors/pixel intensity) and geometric consistency (epipolar geometry).

  #strong[3. Descriptor Matching vs. Optical Flow]
  
  In Deliverable 7, we compared two distinct tracking paradigms:
  - *Descriptor Matching* treats every frame independently. It is robust to drift and can recover from tracking loss (Relocalization), but it is computationally expensive and prone to outliers in repetitive textures.
  - *Lucas-Kanade (LK)* assumes brightness constancy and small motion. It achieved the highest inlier ratio (>97%) and efficiency in our video tests because it searches locally. However, it is susceptible to drift accumulation and cannot recover if the track is lost (e.g., due to rapid motion or occlusion), as it lacks a global descriptor for re-identification.
]

/**************   Conclusion   **************/
= Conclusion
#indent[
  In this lab, we successfully built a modular and functional visual feature tracking frontend. We implemented the full pipeline from feature detection and description to matching and geometric outlier rejection.

  We quantitatively evaluated five different approaches: SIFT, SURF, ORB, FAST+BRIEF, and Lucas-Kanade. Our results demonstrate that there is no single "best" algorithm; rather, the choice depends on the specific application requirements:
  1.  For *offline mapping* or wide-baseline stereo where accuracy and invariance are paramount, *SIFT* or *SURF* are the optimal choices.
  2.  For *real-time drone navigation* or high-speed SLAM where latency is critical and motion is continuous, *ORB* or *Lucas-Kanade* offer the best performance-to-cost ratio.

  Totally, this lab provided a solid foundation for understanding Visual Odometry.
]

#pagebreak()

/**************   Source Code   **************/
#set page(header: none, footer: none) 

= Source Code <section:source_code>
- _*sift_feature_tracker.cpp*_
<section:sift_feature_tracker>
```cpp
#include "sift_feature_tracker.h"

#include <vector>
#include <glog/logging.h>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

using namespace cv;
using namespace cv::xfeatures2d;

/**
   Sift feature tracker Constructor.
*/
SiftFeatureTracker::SiftFeatureTracker()
  : FeatureTracker(),
    detector(SIFT::create()) {}

/** This function detects keypoints in an image.
    @param[in] img Image input where to detect keypoints.
    @param[out] keypoints List of keypoints detected on the given image.
*/
void SiftFeatureTracker::detectKeypoints(const cv::Mat& img,
                                         std::vector<KeyPoint>* keypoints) const {
  CHECK_NOTNULL(keypoints);
  
  detector->detect(img, *keypoints);
}

/** This function describes keypoints in an image.
    @param[in] img Image used to detect the keypoints.
    @param[in, out] keypoints List of keypoints detected on the image. Depending
    on the detector used, some keypoints might be added or removed.
    @param[out] descriptors List of descriptors for the given keypoints.
*/
void SiftFeatureTracker::describeKeypoints(const cv::Mat& img,
                                           std::vector<KeyPoint>* keypoints,
                                           cv::Mat* descriptors) const {
  CHECK_NOTNULL(keypoints);
  CHECK_NOTNULL(descriptors);

  detector->compute(img, *keypoints, *descriptors);
}

/** This function matches descriptors.
    @param[in] descriptors_1 First list of descriptors.
    @param[in] descriptors_2 Second list of descriptors.
    @param[out] matches List of k best matches between descriptors.
    @param[out] good_matches List of descriptors classified as "good"
*/
void SiftFeatureTracker::matchDescriptors(
                                          const cv::Mat& descriptors_1,
                                          const cv::Mat& descriptors_2,
                                          std::vector<std::vector<DMatch>>* matches,
                                          std::vector<cv::DMatch>* good_matches) const {
  CHECK_NOTNULL(matches);
  FlannBasedMatcher flann_matcher;

  flann_matcher.knnMatch(descriptors_1, descriptors_2, *matches, 2);

  const float ratio_thresh = 0.8f;
  for (size_t i = 0; i < matches->size(); i++) {
    if ((*matches)[i].size() >= 2) {
      if((*matches)[i][0].distance < ratio_thresh * (*matches)[i][1].distance) {
        good_matches->push_back((*matches)[i][0]);
      }
    }
  }
}
```

- _*feature_tracker.cpp*_
<section:feature_tracker>
```cpp
#include "feature_tracker.h"

#include <vector>
#include <numeric>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

#include <ros/ros.h>

using namespace cv;
using namespace cv::xfeatures2d;

/** This is the main tracking function, given two images, it detects,
 * describes and matches features.
 * We will be modifying this function incrementally to plot different figures
 * and compute different statistics.
 @param[in] img_1, img_2 Images where to track features.
 @param[out] matched_kp_1_kp_2 pair of vectors of keypoints with the same size
 so that matched_kp_1_kp_2.first[i] matches with matched_kp_1_kp_2.second[i].
*/
void FeatureTracker::trackFeatures(const cv::Mat &img_1,
                                   const cv::Mat &img_2,
                                   std::pair<std::vector<cv::KeyPoint>, std::vector<cv::KeyPoint>> *matched_kp_1_kp_2) {

  std::vector<KeyPoint> keypoints_1, keypoints_2;

  detectKeypoints(img_1, &keypoints_1);
  detectKeypoints(img_2, &keypoints_2);
  
  cv::Mat img_1_kp, img_2_kp;

  cv::drawKeypoints(img_1, keypoints_1, img_1_kp, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
  cv::drawKeypoints(img_2, keypoints_2, img_2_kp, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
  
  cv::imwrite("sift_keypoints_1.png", img_1_kp);
  cv::imwrite("sift_keypoints_2.png", img_2_kp);

  cv::imshow("Keypoints Image 1", img_1_kp);
  cv::imshow("Keypoints Image 2", img_2_kp);
  cv::waitKey(50);
  
  cv::Mat descriptors_1, descriptors_2;
  describeKeypoints(img_1, &keypoints_1, &descriptors_1);
  describeKeypoints(img_2, &keypoints_2, &descriptors_2);
  
  std::vector<std::vector<DMatch>> matches;
  std::vector<DMatch> good_matches;
  
  if (!descriptors_1.empty() && !descriptors_2.empty()) {
      matchDescriptors(descriptors_1, descriptors_2, &matches, &good_matches);
  }
  
  cv::Mat img_matches;
  cv::drawMatches(img_1, keypoints_1, img_2, keypoints_2, good_matches, img_matches, 
                  cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(), 
                  cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
  cv::imwrite("sift_matches.png", img_matches);
  cv::imshow("Matches", img_matches);
  cv::waitKey(0);

  std::vector<cv::KeyPoint> good_kp1, good_kp2;
  for(const auto& m : good_matches) {
      good_kp1.push_back(keypoints_1[m.queryIdx]);
      good_kp2.push_back(keypoints_2[m.trainIdx]);
  }
  
  std::vector<uchar> inlier_mask;
  if (good_matches.size() >= 8) { 
      inlierMaskComputation(good_kp1, good_kp2, &inlier_mask);
  } else {
      inlier_mask.resize(good_matches.size(), 0);
  }
  
  unsigned int num_inliers = 0;
  for(auto mask_val : inlier_mask) {
      if(mask_val) num_inliers++;
  }
  
  cv::Mat img_inliers;
  
  std::vector<char> mask_char(inlier_mask.begin(), inlier_mask.end());
  
  cv::drawMatches(img_1, keypoints_1, img_2, keypoints_2, good_matches, img_inliers,
                  cv::Scalar(0, 255, 0),
                  cv::Scalar(0, 0, 255),
                  mask_char,
                  cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
                  
  cv::imwrite("sift_inliers.png", img_inliers);
  cv::imshow("Inliers", img_inliers);
  cv::waitKey(50);
  
  float const new_num_samples = static_cast<float>(num_samples_) + 1.0f;
  float const old_num_samples = static_cast<float>(num_samples_);
  avg_num_keypoints_img1_ = (avg_num_keypoints_img1_ * old_num_samples + static_cast<float>(keypoints_1.size())) / new_num_samples;
  avg_num_keypoints_img2_ = (avg_num_keypoints_img2_ * old_num_samples + static_cast<float>(keypoints_2.size())) / new_num_samples;
  avg_num_matches_ = (avg_num_matches_ * old_num_samples + static_cast<float>(matches.size())) / new_num_samples;
  avg_num_good_matches_ = (avg_num_good_matches_ * old_num_samples + static_cast<float>(good_matches.size())) / new_num_samples;
  avg_num_inliers_ = (avg_num_inliers_ * old_num_samples + static_cast<float>(num_inliers)) / new_num_samples;
  avg_inlier_ratio_ =
      (avg_inlier_ratio_ * old_num_samples + (static_cast<float>(num_inliers) / static_cast<float>(good_matches.size()))) / new_num_samples;
  ++num_samples_;
}

/** Compute Inlier Mask out of the given matched keypoints.
   *  Both keypoints_1 and keypoints_2 input parameters must be ordered by match
   * i.e. keypoints_1[0] has been matched to keypoints_2[0].
   * Therefore, both keypoints vectors must have the same length.
    @param[in] keypoints_1 List of keypoints detected on the first image.
    @param[in] keypoints_2 List of keypoints detected on the second image.
    @param[out] inlier_mask Mask indicating inliers (1) from outliers (0).
  */
void FeatureTracker::inlierMaskComputation(const std::vector<KeyPoint> &keypoints_1,
                                           const std::vector<KeyPoint> &keypoints_2,
                                           std::vector<uchar> *inlier_mask) const {
  CHECK_NOTNULL(inlier_mask);
  const size_t size = keypoints_1.size();
  CHECK_EQ(keypoints_2.size(), size) << "Size of keypoint vectors "
                                        "should be the same!";

  std::vector<Point2f> pts1(size);
  std::vector<Point2f> pts2(size);
  for (size_t i = 0; i < keypoints_1.size(); i++) {
    pts1[i] = keypoints_1[i].pt;
    pts2[i] = keypoints_2[i].pt;
  }

  static constexpr double max_dist_from_epi_line_in_px = 3.0;
  static constexpr double confidence_prob = 0.99;
  try {
    findFundamentalMat(pts1, pts2, CV_FM_RANSAC, max_dist_from_epi_line_in_px, confidence_prob, *inlier_mask);
  } catch (...) {
    ROS_WARN("Inlier Mask could not be computed, this can happen if there"
             "are not enough features tracked.");
  }
}

/** Example of function to draw matches. Feel free to re-use this example or
 *  create your own. You will need to modify it in order to plot the different
 *  figures. You can add more functions to this class if needed.
 */
void FeatureTracker::drawMatches(const cv::Mat &img_1,
                                 const cv::Mat &img_2,
                                 const std::vector<KeyPoint> &keypoints_1,
                                 const std::vector<KeyPoint> &keypoints_2,
                                 const std::vector<std::vector<DMatch>> &matches) {
  cv::namedWindow("tracked_features", cv::WINDOW_NORMAL);
  cv::Mat img_matches;
  cv::drawMatches(img_1,
                  keypoints_1,
                  img_2,
                  keypoints_2,
                  matches,
                  img_matches,
                  Scalar::all(-1),
                  Scalar::all(-1),
                  std::vector<std::vector<char>>(),
                  DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

  imshow("tracked_features", img_matches);
  
  while (ros::ok() && waitKey(10) == -1) {}
}

void FeatureTracker::printStats() const {
  std::cout << "Avg. Keypoints 1 Size: " << avg_num_keypoints_img1_ << std::endl;
  std::cout << "Avg. Keypoints 2 Size: " << avg_num_keypoints_img2_ << std::endl;
  std::cout << "Avg. Number of matches: " << avg_num_matches_ << std::endl;
  std::cout << "Avg. Number of good matches: " << avg_num_good_matches_ << std::endl;
  std::cout << "Avg. Number of Inliers: " << avg_num_inliers_ << std::endl;
  std::cout << "Avg. Inliers ratio: " << avg_inlier_ratio_ << std::endl;
  std::cout << "Num. of samples: " << num_samples_ << std::endl;
}
```

- _*surf_feature_tracker.cpp*_
<section:surf_feature_tracker>
```cpp
#include "surf_feature_tracker.h"

#include <vector>
#include <glog/logging.h>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

using namespace cv;
using namespace cv::xfeatures2d;

/**
   Surf feature tracker Constructor.
*/
SurfFeatureTracker::SurfFeatureTracker()
  : FeatureTracker(),
    detector(SURF::create()) {

}

/** This function detects keypoints in an image.
    @param[in] img Image input where to detect keypoints.
    @param[out] keypoints List of keypoints detected on the given image.
*/
void SurfFeatureTracker::detectKeypoints(const cv::Mat& img,
                                         std::vector<KeyPoint>* keypoints) const {
  CHECK_NOTNULL(keypoints);

  detector->detect(img, *keypoints);
}

/** This function describes keypoints in an image.
    @param[in] img Image used to detect the keypoints.
    @param[in, out] keypoints List of keypoints detected on the image. Depending
    on the detector used some keypoints might be added or removed.
    @param[out] descriptors List of descriptors for the given keypoints.
*/
void SurfFeatureTracker::describeKeypoints(const cv::Mat& img,
                                           std::vector<KeyPoint>* keypoints,
                                           cv::Mat* descriptors) const {
  CHECK_NOTNULL(keypoints);
  CHECK_NOTNULL(descriptors);
  
  detector->compute(img, *keypoints, *descriptors)
}

/** This function matches descriptors.
    @param[in] descriptors_1 First list of descriptors.
    @param[in] descriptors_2 Second list of descriptors.
    @param[out] matches List of k best matches between descriptors.
    @param[out] good_matches List of descriptors classified as "good"
*/
void SurfFeatureTracker::matchDescriptors(
                                          const cv::Mat& descriptors_1,
                                          const cv::Mat& descriptors_2,
                                          std::vector<std::vector<DMatch>>* matches,
                                          std::vector<cv::DMatch>* good_matches) const {
  CHECK_NOTNULL(matches);

  FlannBasedMatcher flann_matcher;
  // 1. KNN Match (k=2)
  if (!descriptors_1.empty() && !descriptors_2.empty()) {
      flann_matcher.knnMatch(descriptors_1, descriptors_2, *matches, 2);
  }

  // 2. Lowe's Ratio Test
  const float ratio_thresh = 0.8f;
  for (size_t i = 0; i < matches->size(); i++) {
    if ((*matches)[i].size() >= 2) {
      if ((*matches)[i][0].distance < ratio_thresh * (*matches)[i][1].distance) {
        good_matches->push_back((*matches)[i][0]);
      }
    }
  }
}
```

- _*orb_feature_tracker.cpp*_
<section:orb_feature_tracker>
```cpp
#include "orb_feature_tracker.h"
#include <opencv2/features2d.hpp>

using namespace cv;
using namespace cv::xfeatures2d;

OrbFeatureTracker::OrbFeatureTracker()
    : detector(ORB::create(500, 1.2f, 1)) {}

void OrbFeatureTracker::detectKeypoints(const cv::Mat &img,
                                         std::vector<cv::KeyPoint> *keypoints) const {
    detector->detect(img, *keypoints);
}

void OrbFeatureTracker::describeKeypoints(const cv::Mat &img,
                                           std::vector<cv::KeyPoint> *keypoints,
                                           cv::Mat *descriptors) const {
    detector->compute(img, *keypoints, *descriptors);
}

void OrbFeatureTracker::matchDescriptors(const cv::Mat &descriptors_1,
                                          const cv::Mat &descriptors_2,
                                          std::vector<std::vector<cv::DMatch>> *matches,
                                          std::vector<cv::DMatch> *good_matches) const {
    cv::BFMatcher matcher(cv::NORM_HAMMING);
    
    if (!descriptors_1.empty() && !descriptors_2.empty()) {
        matcher.knnMatch(descriptors_1, descriptors_2, *matches, 2);
    }

    // Ratio Test
    const float ratio_thresh = 0.8f;
    for (size_t i = 0; i < matches->size(); i++) {
        if ((*matches)[i].size() >= 2) {
            if ((*matches)[i][0].distance < ratio_thresh * (*matches)[i][1].distance) {
                good_matches->push_back((*matches)[i][0]);
            }
        }
    }
}
```

- _*fast_feature_tracker.cpp*_
<section:fast_feature_tracker>
```cpp
#include "fast_feature_tracker.h"
#include <opencv2/features2d.hpp>

#include <opencv2/xfeatures2d.hpp>

using namespace cv;
using namespace cv::xfeatures2d;

FastFeatureTracker::FastFeatureTracker()
    : detector(FastFeatureDetector::create()) {}

void FastFeatureTracker::detectKeypoints(const cv::Mat &img,
                                         std::vector<cv::KeyPoint> *keypoints) const {
    detector->detect(img, *keypoints);
}

void FastFeatureTracker::describeKeypoints(const cv::Mat &img,
                                           std::vector<cv::KeyPoint> *keypoints,
                                           cv::Mat *descriptors) const {
    Ptr<BriefDescriptorExtractor> extractor = BriefDescriptorExtractor::create();
    extractor->compute(img, *keypoints, *descriptors);
}

void FastFeatureTracker::matchDescriptors(const cv::Mat &descriptors_1,
                                          const cv::Mat &descriptors_2,
                                          std::vector<std::vector<cv::DMatch>> *matches,
                                          std::vector<cv::DMatch> *good_matches) const {
    cv::BFMatcher matcher(cv::NORM_HAMMING);
    
    if (!descriptors_1.empty() && !descriptors_2.empty()) {
        matcher.knnMatch(descriptors_1, descriptors_2, *matches, 2);
    }

    // Ratio Test
    const float ratio_thresh = 0.8f;
    for (size_t i = 0; i < matches->size(); i++) {
        if ((*matches)[i].size() >= 2) {
            if ((*matches)[i][0].distance < ratio_thresh * (*matches)[i][1].distance) {
                good_matches->push_back((*matches)[i][0]);
            }
        }
    }
}
```

- _*lk_feature_tracker.cpp*_
<section:lk_feature_tracker>
```cpp
#include "lk_feature_tracker.h"

#include <numeric>
#include <vector>

#include <glog/logging.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#include <ros/ros.h>

using namespace cv;
using namespace cv::xfeatures2d;

/**
   LK feature tracker Constructor.
*/
LKFeatureTracker::LKFeatureTracker() {
  // Feel free to modify if you want!
  cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
}

LKFeatureTracker::~LKFeatureTracker() {
  // Feel free to modify if you want!
  cv::destroyWindow(window_name_);
}

/** This is the main tracking function, given two images, it detects,
 * describes and matches features.
 * We will be modifying this function incrementally to plot different figures
 * and compute different statistics.
 @param[in] frame Current image frame
*/
void LKFeatureTracker::trackFeatures(const cv::Mat& frame) {
  // 1. Initialize the keypoint container for the current frame
  std::vector<cv::Point2f> curr_points;
  std::vector<uchar> status;
  std::vector<float> err;

  // 2. Logical branch: initialization vs tracing
  if (prev_corners_.empty()) {
    cv::goodFeaturesToTrack(frame, prev_corners_, 1000, 0.01, 10);
    
    frame.copyTo(prev_frame_);
  } else {
    cv::calcOpticalFlowPyrLK(prev_frame_, frame, prev_corners_, curr_points, 
                             status, err, cv::Size(21, 21), 3);

    // 3. Filter out lost points (status == 0) and out-of-bounds points
    std::vector<cv::Point2f> good_prev;
    std::vector<cv::Point2f> good_curr;

    for (size_t i = 0; i < status.size(); i++) {
        if (status[i]) {
            good_prev.push_back(prev_corners_[i]);
            good_curr.push_back(curr_points[i]);
        }
    }

    // 4. Calculate and print the statistics
    std::vector<uchar> inlier_mask;
    inlierMaskComputation(good_prev, good_curr, &inlier_mask);
    
    int inlier_count = 0;
    for(auto val : inlier_mask) if(val) inlier_count++;

    // Log (for form filling)
    static int frame_count = 0;
    frame_count++;
    if (frame_count % 10 == 0) {
        ROS_INFO_STREAM("LK Stats: Matches: " << good_curr.size() 
                        << " Inliers: " << inlier_count 
                        << " Ratio: " << (double)inlier_count / good_curr.size());
    }

    // 5. Visualization
    cv::Mat img_viz;
    cv::cvtColor(frame, img_viz, cv::COLOR_GRAY2BGR);
    show(img_viz, good_prev, good_curr);

    // 6. Compensate Mechanism
    if (good_curr.size() < 800) {
        std::vector<cv::Point2f> new_points;
        cv::Mat mask = cv::Mat::zeros(frame.size(), CV_8UC1); 
        mask.setTo(cv::Scalar(255));
        for(auto& p : good_curr) cv::circle(mask, p, 10, cv::Scalar(0), -1);

        cv::goodFeaturesToTrack(frame, new_points, 1000 - good_curr.size(), 0.01, 10, mask);
        good_curr.insert(good_curr.end(), new_points.begin(), new_points.end());
    }

    // 7. Refresh stats
    prev_corners_ = good_curr;
    frame.copyTo(prev_frame_);
  }
}

/** Display image with tracked features from prev to curr on the image
 * corresponding to 'frame'
 * @param[in] frame The current image frame, to draw the feature track on
 * @param[in] prev The previous set of keypoints
 * @param[in] curr The set of keypoints for the current frame
 */
void LKFeatureTracker::show(const cv::Mat& frame, std::vector<cv::Point2f>& prev,
                            std::vector<cv::Point2f>& curr) {
  cv::Mat viz_img = frame.clone();

  for (size_t i = 0; i < curr.size(); i++) {
      cv::line(viz_img, prev[i], curr[i], cv::Scalar(0, 255, 0), 2);
      cv::circle(viz_img, curr[i], 3, cv::Scalar(0, 0, 255), -1);
  }

  cv::imshow(window_name_, viz_img);
  cv::waitKey(1);
}

/** Compute Inlier Mask out of the given matched keypoints.
 @param[in] pts1 List of keypoints detected on the first image.
 @param[in] pts2 List of keypoints detected on the second image.
 @param[out] inlier_mask Mask indicating inliers (1) from outliers (0).
*/
void LKFeatureTracker::inlierMaskComputation(const std::vector<cv::Point2f>& pts1,
                                             const std::vector<cv::Point2f>& pts2,
                                             std::vector<uchar>* inlier_mask) const {
  CHECK_NOTNULL(inlier_mask);

  static constexpr double max_dist_from_epi_line_in_px = 3.0;
  static constexpr double confidence_prob = 0.99;
  try {
    findFundamentalMat(pts1, pts2, CV_FM_RANSAC,
                       max_dist_from_epi_line_in_px, confidence_prob,
                       *inlier_mask);
  } catch(...) {
    ROS_WARN("Inlier Mask could not be computed, this can happen if there"
             "are not enough features tracked.");
  }
}
```

- _*self_flow.cpp*_
<section:self_flow>
```cpp
#include <memory>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

using namespace std;

/** imageCallback This function is called when a new image is published. */
void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
  try {
    cv::Mat image = cv_bridge::toCvCopy(msg, "bgr8")->image;

    cv::Mat curr_gray;
    cv::cvtColor(image, curr_gray, cv::COLOR_BGR2GRAY);

    static cv::Mat prev_gray;

    if (prev_gray.empty()) {
      curr_gray.copyTo(prev_gray);
      return;
    }

    cv::Mat flow;
    cv::calcOpticalFlowFarneback(prev_gray, curr_gray, flow, 0.5, 3, 15, 3, 5, 1.2, 0);

    std::vector<cv::Mat> flow_parts;
    cv::split(flow, flow_parts);
    
    cv::Mat magnitude, angle;
    cv::cartToPolar(flow_parts[0], flow_parts[1], magnitude, angle, true);

    std::vector<cv::Mat> hsv_planes(3);

    angle.convertTo(hsv_planes[0], CV_8UC1, 0.5);

    hsv_planes[1] = cv::Mat(angle.size(), CV_8UC1, cv::Scalar(255));

    cv::Mat v_norm;
    cv::normalize(magnitude, v_norm, 0, 255, cv::NORM_MINMAX);
    v_norm.convertTo(hsv_planes[2], CV_8UC1);

    cv::Mat hsv;
    cv::merge(hsv_planes, hsv); 

    cv::Mat result_bgr;
    cv::cvtColor(hsv, result_bgr, cv::COLOR_HSV2BGR);

    cv::imshow("view", result_bgr);
    cv::waitKey(1);

    curr_gray.copyTo(prev_gray);

  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
}

/**
 * @function main
 * @brief Main function
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "optical_flow");
  ros::NodeHandle local_nh("~");

  cv::namedWindow("view", cv::WINDOW_NORMAL);
  image_transport::ImageTransport it(local_nh);
  image_transport::Subscriber sub = it.subscribe("/images_topic", 100, imageCallback);

  ros::spin();
  cv::destroyWindow("view");
  return EXIT_SUCCESS;
}
```