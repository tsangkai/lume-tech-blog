---
title: Building a library for auto-differentiation on Lie groups
date: 2025-10-26
author: Tsang-Kai Chang
tags:
  - auto differentiation
  - Lie group
  - ICP
---

The [previous post](./icp-point-to-plane.md) derives the plane-to-point
ICP algorithm with Lie group. However, in the implementation, one has to explicitly
specify how to calculate the gradient. In this post, I use the same ICP algorithm
as an example to show an auto-differentiation library. With this library,
one only has to explicitly define the function and specify the variable, and the gradient is then provided by
the library without any additional code from the user.

<!--more-->

The design can be divided into 2 parts. The first part defines the Lie group and
Lie algebra classes, and the operations on them. The second part builds on top
of the first part, and provides gradient for each operations. This separation
provides the flexibility. For example, when the gradient is not needed, the
second part does not involve.

## ICP as an example

To demonstrate this library, we only focus on the transformation optimization step in the plane-to-point version of ICP.
The problem is as follow: given the points \\( s_i \\) from the source point cloud and the corresponding points and normals \\( \\{t_i, n_i\\} \\)
from the target point cloud, we want to find the optimal transformation \\( T \\). Formally,
$$
\min_{T \in SE(3)}
\sum_{i}|| f(T) ||^2, \quad f(T) = n_i^{\mathsf{T}} \left(T(s_i) - d_i \right).
$$

This is a least sqaures problem, and thus the gradient of \\( f \\) w.r.t. \\( T \\) is sufficient to solve the optimization problem iteratively.

## Lie group and Lie algebra

We first have to define some Lie group classes to characterize varaibles, the transformation \\( T \\) for example.
There are 3 types of Lie groups in \\( f(T) \\), including \\( \mathbb{R} \\), \\( \mathbb{R}^3 \\), and \\( SE(3) \\).
As an example, the class of \\( SE(3) \\) along with some operations can be easily defined as

```cpp
namespace lie {

struct SE3 {
    static constexpr int Dim = 6;
    using DataType = Eigen::Transform<double, 3, Eigen::Isometry>;

    DataType data;

    SE3() = default;
    SE3(DataType data) : data(data) {}
};

inline SE3 operator*(const SE3 &t1, const SE3 &t2) {
    return SE3::DataType{t1.data * t2.data};
}

inline R3 operator*(const SE3 &t, const R3 &v) { return {t.data * v.data}; }

}    // namespace lie
```

In `lie::SE3`, I just use `Eigen::Transform` as the underlying data.
In this way, a lot of operations are already provided by `Eigen`.

## Auto-differentiation

To enable the auto-differentiation, I add a wrapper to distinguish between constants and variables in a function.
For example, \\( s_i \\) is a constant in \\( f(T) \\), and \\( T \\) of course is a variable. The main difference is that
variables come with gradients while constants don't.

The sample code looks like

```cpp
namespace autoDiff {

template <class Value, int N> struct Variable {
    using GradientType = Eigen::Matrix<double, Value::Dim, N>;

    Value value;
    GradientType grad;
};

template <class Value> struct Constant {
    Value value;
};

}    // namespace autoDiff
```

In the class `Variable`, the first template parameter `Value` specifies type of this variable, and the class
member `grad` is the graident accumulated up to the current varaible. The
second template parameter `N` is the dimension of the variable that we differentiate against.

We now have to define how is gradient calculated with `Variable`s and `Constant`s.
Take the first operation in \\( f(T) \\) as an example. With \\( f_1(T) = T(s_i) \in \mathbb{R} ^3\\), we
want to get the gradient of \\( f_1 \\) with respect to \\( T \\). In this case, \\( f_1 \\) is also a
`Variable`, and its `N` will be the dimension of `T`.

```cpp
namespace autoDiff {

template <int N>
Variable<lie::R3, N> operator*(const Variable<lie::SE3, N> &T,
                               const Constant<lie::R3> &v) {
    auto const &R = T.value.data.rotation();
    auto grad = Eigen::Matrix<double, 3, 6>::Zero().eval();
    grad.template block<3, 3>(0, 0) = R;
    grad.template block<3, 3>(0, 3) = -R * lie::crossFunc(v.value.data);

    return {T.value * v.value, grad};
}

}    // namespace autoDiff
```

The gradient calculation is from (182) [Sola2018]. Note the the value of the varible is calculated in the Lie group scope.
The auto-differentiation library only have to provide the gradient calculation.

## Usage

To use this library to calculate the differentiation automatically, a user just has to
provide a functor specifying the input and output varaibles.

``` cpp
struct IcpResidualFunctor {
    IcpResidualFunctor(const lie::R3::DataType &source,
                       const lie::R3::DataType &target,
                       const lie::R3::DataType &normal)
        : source_(source), target_(target), normal_(normal) {}

    // the first template parameter: the type of the output
    // the second template parameter: the dimesion of the variable
    autoDiff::Variable<lie::R1, lie::SE3::Dim>
    operator()(const lie::SE3 &transform) const {
        const auto transformVariable = autoDiff::Variable<lie::SE3, lie::SE3::Dim>(
            transform);
        return autoDiff::dot(transformVariable * source_ - target_, normal_);
    }

    autoDiff::Constant<lie::R3> source_;
    autoDiff::Constant<lie::R3> target_;
    autoDiff::Constant<lie::R3> normal_;
};
```

This functor does not expose the auto-differentiation part in the interface.
The auto-differentiation is taking place inside the function.
Inside `operator()(...)`, we first create a `Varaibel`. As this is the differentiation
variable, the parameter `N` is its dimension and the gradient will be set to an
identity matrix.

With the functor defined, one only has to use the following code
to get the function value and gradient at once.

``` cpp
auto const icpResidualFunctor =
    IcpResidualFunctor{sourcePointCloud.points[sourceIdx],
                       targetPointCloud.points[targetIdx],
                       targetPointCloud.normals[targetIdx]};
auto result = icpResidualFunctor(T);

auto const &graident = result.grad;
auto const &functionValue = result.value.data;
```

## Summary

The post summarizes a prototype of an auto-differentiation library, which can take variables
in Lie group. This library has a clear boundary between the value calculation and the gradient
calculation, which provides the flexibility.

## Reference

[Sola2018] J. Sola, J. Deray, and D. Atchuthan, “A micro Lie theory for state
estimation in robotics,” 2018.
