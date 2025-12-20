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
  title: "Lab 5 Report",
  course: "Robotics Integration Group Project I",
  author-name: "Yuwei ZHAO",
  author-id: "23020036096",
  group: "Group #31",
  date: "2025-12-20",
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

=== Practice with Perspective Projection

#boldify[
  Consider a sphere with radius $r$ centered at $[0, 0, d]$ with respect to the camera coordinate frame (centered at the optical center and with axis oriented as discussed in class). Assume $d > r + 1$ and assume that the camera has principal point at (0, 0), focal length equal to 1, pixel sizes $s_x = s_y = 1$ and zero skew $s_theta = 0$ the following exercises:
]

#boldify[
  1. Derive the equation describing the projection of the sphere onto the image plane.
]

#indent[
  *Solution:* \
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