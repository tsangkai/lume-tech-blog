---
title: Using Lie Group to Derive Point-to-Plane ICP Algorithm
date: 2025-05-13
author: Tsang-Kai Chang
tags:
  - ICP
  - nonlinear least sqaures
  - Lie group
---

Given 2 point clouds, the Iterative Closest Point (ICP) algorithm can find a
transform such that one after the transform is well-algined to the other.
Generally, the ICP algorithm iterates between the step to find correspondence
and the step to find the optimal transformation, and the later step is usually
modeled as a nonlinear least squares problem. This post uses the point-to-plane
version of ICP to show how to derive the ICP algorithm elegantly with the help
of Lie group framework.

<!--more-->

## Nonlinear Least Squares Problem

Suppose that we have to solve an optimization problem where the paramter \\( X
\\) lies on a manifold \\( \mathcal{M} \\), or $$
   \min_{X \in \mathcal{M}} || f(X)||^2,
   $$
where \\( f:\mathcal{M} \to \mathbb{R}^n \\).

In general, there is no explicit solution for this problem. Instead, we take
advantage of the fact the manifold looks linear locally, and approach this
problem iteratively.

To begin with, for \\( X \\) around \\( X_k \\), we have $$
   X = X_k \oplus w_k,
$$
where \\( w_k \in T_{X_k}\mathcal{M} \\) and \\( \oplus \\) is the right-plus
defined in [Sola2018]. \\( k \\) is the iteration index. Then \\( f(X) \\) can
thus be approximated as $$
      f(X) = f(X_k \oplus x_k) \approx f(X_k) + J_{X_k} w_k.
   $$

This is just eq. (43) in [Sola2018], and \\( J_{X_k} \in \mathbb{R}^{n \times m}
\\) is the Jacobian matrix. This approximation suggests that instead of finding
the optimal \\( X \\), the optimization goal is now to find the optimal \\( w_k
\\) in the tangent space of \\( T_{X_k} \mathcal{M} \\).

Furthremore, the approximated optimization is quadratic in \\(w_k\\), which has
an explicit solution. In detail, as the approximated optimization problem is
given by $$
   \min_{x_k \in \mathbb{R}^m } || f(X_k) + J_{X_k} w_k ||^2,
   $$
we take derivative of the squared term with respect to \\( w_k \\) and set it to
\\( 0 \\), and we get $$
      \left( f(X_k) + J_{X_k} w_k \right)^{\mathsf{T}} J_{X_k} = 0.
   $$
\\( w_k \\) can then be obtained by solving the linear equation: $$
      J_{X_k}^{\mathsf{T}} J_{X_k} w_k = -J_{X_k}^{\mathsf{T}} f(X_k).
   $$

Returning back to the original optimization problem, we get a new solution of
\\(X \\) by updating $$
      X_{k+1} = X_k \oplus w_k.
   $$

Note that this derivation can be easily extended to the optimization problem $$
      \min_{X \in \mathcal{M}} \sum_{i}|| f_i(X)||^2.
   $$

## Point-to-Plane ICP

In general, an ICP algorithm iterates between 2 steps:
- finding the correspondence set \\( C \\), given the current transformation \\(
  T \\)
- finding the optimal transform \\( T \\) from the source point cloud to the
  targe point cloud given \\( C \\)

In point-to-plane ICP, the second step is formulated as $$
      \min_{T \in SE(3)} \sum_{i}||  n_i^{\mathsf{T}} \left(T(s_i) - d_i \right) ||^2.
   $$
\\( s_i \\) and \\( d_i \\) denote the 3d point of the $i$th correspondence pair
in the source point cloud and the target point cloud respectively.

With \\( f(T) = n_i^{\mathsf{T}} \left(T(s_i) - d_i \right) \\), we can first
observe \\( f = h \circ g \\), with $$
      g_i(T) = T(s_i) - d_i, \quad h_i(v) = n_i^{\mathsf{T}} v.
   $$

Based on [Sola2018], we have
```math
J^f_T = J^h_{g(T)} J^g_{T} = n_i^{\mathsf{T}} 
\begin{bmatrix}
   R & -R [t]_{\times}
\end{bmatrix},
\quad
T = 
\begin{bmatrix}
   R & t \\
   0 & 1
\end{bmatrix}.
```
The first Jacobian matrix of inner product is easy, and the second is given in
(182).

## Sample Code

The following code sample shows how to use the Lie group operation to realize
finding transformation in the point-to-plane ICP algorithm. The part of finding
correspondence set is less related to the Lie group formulation, and thus it is
omitted here. In the code, we use
[the library `manif`](https://github.com/artivis/manif), which is developed by
the authors of [Sola2018].

In the code sample, we do not have to operate complicated math explicitly, but
instead take advantage of the well-established Lie group framework.

```cpp
#include <manif/SE3.h>

using CorrespondenceSet = std::vector<std::pair<size_t, size_t>>;
using Transformation = Eigen::Transform<double, 3, Eigen::Isometry>;

Transformation findTransformation(
    const PointCloud &sourcePointCloud,
    const PointCloud &targetPointCloud,
    const Transformation &transformation,
    const CorrespondenceSet &correspondenceSet) {

    // The linear equation: A X = b
    auto A = Eigen::Matrix<double, 6, 6>::Zero().eval();
    auto b = Eigen::Matrix<double, 6, 1>::Zero().eval();

    auto const T = manif::SE3d{transformation};

    for (auto const &[sourceIdx, targetIdx] : correspondenceSet) {

        auto const &sourcePoint = sourcePointCloud.points[sourceIdx];
        auto const &targetPoint = targetPointCloud.points[targetIdx];
        auto const targetNormalT =
            targetPointCloud.normals[targetIdx].transpose();

        // calculate the Jacobian matrices
        auto J_transform = Eigen::Matrix<double, 3, 6>{};
        auto const transformedSourcePoint = T.act(sourcePoint, J_transform);

        auto const &J_product = targetNormalT;

        auto const J = J_product * J_transform;
        auto const f = targetNormalT * (transformedSourcePoint - targetPoint);

        A += (J.transpose() * J) / (correspondenceSet.size());
        b += -(J.transpose() * f) / (correspondenceSet.size());
    }

    auto const dT = manif::SE3Tangentd{A.ldlt().solve(b)};

    return T.rplus(dT).isometry();
}
```

## Reference

[Sola2018] J. Sola, J. Deray, and D. Atchuthan, “A micro Lie theory for state
estimation in robotics,” 2018.
