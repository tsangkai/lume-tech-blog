---
title: The Derivation of Continuous-Time Factor Graph Optimization 
date: 2025-02-14
author: Tsang-Kai Chang
tags:
  - robotics
  - factor graph
---

One of the main challenges when using factor graph optimizer is the synchronization between keyframes and measurements. At the early stage of development, there are only camera and IMU measurement, and one can easily synchronize the keyframe to the camera data. Once we start to use different kinds of sensors and their data come at different time, the synchronization becomes an issue.
   
I recently found a paper that explicitly addresses this issue [Zhang2024]. However, I have some problem understanding the derivation. This note then summarizes the main theoretical foundation of the method with some comments.

<!--more-->

# Derivation

## Linear Stochastic Equation

The Gaussian process regression assumes that the system dynamics follows linear stochastic differential equation
   $$
      \dot{\gamma}(t) = A \gamma(t) + F w(t),
   $$
   where \\( w(t) \\) follows a zero-mean Gaussian process.
   
   Given \\( \gamma(t_i) \\) and \\( \gamma(t_j) \\), the Gaussian regression interpolates \\( \gamma(t_\tau) \\), \\( t_i \leq t_\tau = t_i + \tau \leq t_j = t_i + \Delta t \\), as
   $$
      \gamma(t_\tau) = \Lambda(t_\tau) \gamma(t_i) + \Omega(t_\tau) \gamma(t_j),
   $$
   where
   $$
      \Lambda(t_{\tau}) = \Phi(\tau) - \Omega(t_\tau) \Phi(\Delta t), 
   $$
   and
   $$
      \Omega(t_\tau) = Q(\tau) \Phi(t_j - t_\tau)^{\mathsf{T}} Q^{-1}(\Delta t).
   $$

   The state transition matrix  \\( \Phi \\) and the covariance matrix \\( Q \\) depend on the underlying model.
   
   This part of derivation can be found in [Barfoot2024]. 
   
## Local Pose Representation

The Gaussian regression requires that the system dynamics to be linear. However, the space where the pose we are interested in resides is not linear. We then express the pose, the velocity and the acceleration around \\( T_{t_i} \\) locally to ensure the linearity.
   
First, we use \\( \varpi \in \mathbb{R}^6 \\) and \\( \dot{\varpi} \in \mathbb{R}^6 \\) to denote the body-centric velocity and acceleration. The pose and the velocity is related by
$$
      \dot{T}(t) = T(t) \varpi^{\wedge}(t).
$$

Note that in some paper, the velocity is left-multiplied. However, the equation (6.101) in [Barfoot2022] indicates that the right-multiplication should be the correct one. Please refer to Eq. (2.56) in [Murray1994] for a more detailed interpretation of \\( \varpi \\) from the screw theory.
   
We then define the local variable \\( \xi(t) \\) as
$$
      \xi(t) = \log(T_{t_i}^{-1} T(t))^{\vee}.
$$
\\( \xi(t) \\) can be regarded as the additional input coming from the right multiplication to \\( T_{t_i} \\).
   
For the first order derivative \\( \dot{\xi}(t) \\), we can use the relation
   $$
      T^{-1}(t) \dot{T}(t) = \left( J_r(\xi(t)) \dot{\xi}(t) \right)^{\wedge},
   $$
   where \\( J_r \\) is the right-Jacobian, to obtain
   $$
      \dot{\xi}(t) = J_r(\xi(t))^{-1} \varpi(t).
   $$
   This part of derivation is given in [Dong2018].
   
   For the second order derivative \\( \ddot{\xi}(t) \\), we first use the approximation
   $$
      J_r^{-1}(\xi) \approx 1 + \frac{1}{2} \xi^{\curlywedge},
   $$
   to have
   $$
      \frac{d}{dt} \left( J_r^{-1}(\xi) \right)\approx \frac{1}{2} \dot{\xi}^{\curlywedge}.
   $$
   
   The second derivative is then approximated by
   $$
      \ddot{\xi} \approx \frac{1}{2} \left[ J_r(\xi)^{-1} \varpi \right]^{\curlywedge} \varpi + J_r(\xi)^{-1} \dot{\varpi}.
   $$
   
   This derivation can be found in [Tang2019].

## White-Noise-on-Jerk (WNOJ) Motion Prior

The white-noise-on-jerk prior assumes that the third derivative of \\( \xi(t) \\) follows a white noise. This part of derivation can be found in [Tang2019].
 
# Reference

[Barfoot2022] T. D. Barfoot, *State Estimation for Robotics*, 2022.

[Barfoot2024] T. D. Barfoot, C. H. Tong, and S. Sarkka, “Batch continuous-time trajectory estimation as exactly sparse Gaussian process regression,” in *Robotics: Science and Systems*, 2014.

[Dong2018] J. Dong, M. Mukadam, B. Boots, and F. Dellaert, “Sparse Gaussian processes on matrix Lie groups: A unified framework for optimizing continuous-time trajectories,” in ICRA, 2018.

[Murray1994] R. M. Murray, S. S. Sastry, and L. Zexiang, *A Mathematical Introduction to Robotic Manipulation*, 1994.

[Tang2019] T. Y. Tang, D. J. Yoon, and T. D. Barfoot, “A white-noise-on-jerk motion prior for continuous-time trajectory estimation on SE(3),” *IEEE Robotics and Automation Letters*, 2019.

[Zhang2024] H. Zhang, C.-C. Chen, H. Vallery, and T. D. Barfoot, “GNSS/multisensor fusion using continuous-time factor graph optimization for robust localization,” *IEEE Transactions on Robotics*, 2024.

